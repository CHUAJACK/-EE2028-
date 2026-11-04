#!/usr/bin/env python3
"""EE2028 drone dashboard.

Connects to the B-L4S5I-IOT01A board over Bluetooth LE and shows its live
telemetry and laser status in a window.

Setup (once):
    python3 -m venv --system-site-packages host/.venv
    host/.venv/bin/pip install -r host/requirements.txt

Run:
    host/.venv/bin/python host/drone_dashboard.py                 # scan for EE2028-Drone
    host/.venv/bin/python host/drone_dashboard.py --address C0:..  # connect to a known address
    host/.venv/bin/python host/drone_dashboard.py --demo           # fake data, no board needed

The GATT layout must match Core/Inc/ble_drone.h in the firmware.
"""

import argparse
import asyncio
import math
import queue
import random
import struct
import threading
import time
import tkinter as tk

DEVICE_NAME = "EE2028-Drone"


def _uuid(char_id):
    return f"e20280{char_id:02x}-6c61-7365-7200-647230ee2028"


SERVICE_UUID = _uuid(0)
SENSORS_UUID = _uuid(1)
STATUS_UUID = _uuid(2)
LASER_UUID = _uuid(3)

LASER_EVT_NONE, LASER_EVT_FIRED, LASER_EVT_NO_POWER = 0, 1, 2

MODE_DESTROYED = 100   # the board switches Bluetooth off after sending this

# Mode number -> (label, badge colour)
MODES = {
    0: ("STANDBY", "#2e7d32"),
    1: ("BATTLE", "#e65100"),
    2: ("LAST OF 2028", "#c62828"),
    MODE_DESTROYED: ("DESTROYED", "#616161"),
}

# (key, label, unit, format, bit in the out-of-range mask)
SENSORS = [
    ("temp", "Temperature", "°C", "{:.2f}", 0),
    ("pressure", "Pressure", "hPa", "{:.1f}", 1),
    ("humidity", "Humidity", "%RH", "{:.2f}", 2),
    ("accel", "Acceleration", "g", "{:.3f}", 3),
    ("gyro", "Gyroscope", "rad/s", "{:.3f}", 4),
    ("mag", "Magnetometer", "gauss", "{:.3f}", 5),
]

SENSORS_FMT = "<hHHHHHB"   # 13 bytes
STATUS_FMT = "<Bhhh"       # 7 bytes
LASER_FMT = "<BBBBB"       # 5 bytes

SENSOR_STALE_S = 3.0


# ---------------------------------------------------------------- decoding

def decode_sensors(data):
    t, p, h, a, g, m, mask = struct.unpack(SENSORS_FMT, bytes(data[:struct.calcsize(SENSORS_FMT)]))
    return {"temp": t / 100, "pressure": p / 10, "humidity": h / 100,
            "accel": a / 1000, "gyro": g / 1000, "mag": m / 1000, "mask": mask}


def decode_status(data):
    mode, roll, pitch, yaw = struct.unpack(STATUS_FMT, bytes(data[:struct.calcsize(STATUS_FMT)]))
    return {"mode": mode, "roll": roll / 100, "pitch": pitch / 100, "yaw": yaw / 100}


def decode_laser(data):
    charge, max_charge, cost, per_press, event = struct.unpack(LASER_FMT, bytes(data[:struct.calcsize(LASER_FMT)]))
    return {"charge": charge, "max": max_charge, "cost": cost, "per_press": per_press, "event": event}


def encode_sensors(temp, pressure, humidity, accel, gyro, mag, mask):
    return struct.pack(SENSORS_FMT, round(temp * 100), round(pressure * 10), round(humidity * 100),
                       round(accel * 1000), round(gyro * 1000), round(mag * 1000), mask)


def encode_status(mode, roll, pitch, yaw):
    return struct.pack(STATUS_FMT, mode, round(roll * 100), round(pitch * 100), round(yaw * 100))


def encode_laser(charge, max_charge, cost, per_press, event):
    return struct.pack(LASER_FMT, charge, max_charge, cost, per_press, event)


CHARACTERISTICS = [
    (SENSORS_UUID, "sensors", decode_sensors),
    (STATUS_UUID, "status", decode_status),
    (LASER_UUID, "laser", decode_laser),
]


# ---------------------------------------------------------------- data sources
# Both sources put messages on a queue that the window reads:
#   ("link", text, connected)        connection status line
#   (kind, values, is_initial_read)  kind is "sensors", "status" or "laser"

class BleWorker(threading.Thread):
    """Finds the board, subscribes to its characteristics and reconnects on drop-outs."""

    def __init__(self, out_q, name, address):
        super().__init__(daemon=True)
        self.q = out_q
        self.name = name
        self.address = address
        self.loop = None
        self.stop_event = None
        self.stopping = False
        self.board_destroyed = False   # last status said DESTROYED, so the board's Bluetooth is off

    def run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.stop_event = asyncio.Event()
        if self.stopping:
            return
        self.loop.run_until_complete(self._main())

    def stop(self):
        self.stopping = True
        if self.loop and self.stop_event:
            self.loop.call_soon_threadsafe(self.stop_event.set)

    def _link(self, text, connected=False):
        self.q.put(("link", text, connected))

    def _publish(self, kind, values, initial):
        if kind == "status":
            self.board_destroyed = values["mode"] == MODE_DESTROYED
        self.q.put((kind, values, initial))

    async def _find_device(self, BleakScanner, attempt):
        if self.board_destroyed:
            hint = "  (the drone was destroyed and turned its Bluetooth off: reset the board)"
        elif attempt > 1:
            hint = f"  (not found yet, attempt {attempt}: is the board running?)"
        else:
            hint = ""
        if self.address:
            self._link(f"Looking for {self.address}…{hint}")
            return await BleakScanner.find_device_by_address(self.address, timeout=10)

        self._link(f"Scanning for {self.name}…{hint}")

        def matches(device, adv):
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            return self.name in (device.name, adv.local_name) or SERVICE_UUID in uuids

        return await BleakScanner.find_device_by_filter(matches, timeout=10)

    async def _wait_or_stop(self, seconds):
        try:
            await asyncio.wait_for(self.stop_event.wait(), timeout=seconds)
        except asyncio.TimeoutError:
            pass

    async def _main(self):
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError:
            self._link("bleak is not installed: run  pip install -r host/requirements.txt")
            return

        attempt = 0
        while not self.stop_event.is_set():
            try:
                attempt += 1
                device = await self._find_device(BleakScanner, attempt)
                if self.stop_event.is_set():
                    break
                if device is None:
                    continue   # scanning again right away; the status line shows the attempt count

                attempt = 0
                self._link(f"Connecting to {device.address}…")
                dropped = asyncio.Event()
                async with BleakClient(device, disconnected_callback=lambda _c: dropped.set()) as client:
                    self.board_destroyed = False
                    # Read the current values first, then follow changes through notifications
                    for uuid, kind, decode in CHARACTERISTICS:
                        try:
                            self._publish(kind, decode(await client.read_gatt_char(uuid)), True)
                        except Exception:
                            pass
                    for uuid, kind, decode in CHARACTERISTICS:
                        await client.start_notify(uuid, self._on_notify(kind, decode))

                    self._link(f"Connected to {device.name or self.name} ({device.address})", True)
                    waits = [asyncio.ensure_future(self.stop_event.wait()),
                             asyncio.ensure_future(dropped.wait())]
                    await asyncio.wait(waits, return_when=asyncio.FIRST_COMPLETED)
                    for w in waits:
                        w.cancel()

                if not self.stop_event.is_set():
                    self._link("Drone destroyed: it has turned its Bluetooth off. Reset the board to reconnect."
                               if self.board_destroyed else "Connection lost. Reconnecting…")
            except Exception as exc:
                self._link(f"Bluetooth error: {exc}. Retrying…")
            await self._wait_or_stop(2)

    def _on_notify(self, kind, decode):
        def callback(_char, data):
            try:
                self._publish(kind, decode(data), False)
            except struct.error:
                pass   # wrong length; ignore this packet
        return callback


class DemoWorker(threading.Thread):
    """Fake board: cycles through the modes and fires the laser, using the real packet format."""

    def __init__(self, out_q):
        super().__init__(daemon=True)
        self.q = out_q
        self.stopping = False

    def stop(self):
        self.stopping = True

    def run(self):
        self.q.put(("link", "Demo mode (fake data, no Bluetooth)", True))
        charge = 0
        self.q.put(("laser", decode_laser(encode_laser(charge, 10, 5, 3, LASER_EVT_NONE)), True))
        start = time.monotonic()
        last_sensors = last_press = 0.0
        while not self.stopping:
            now = time.monotonic()
            t = (now - start) % 30   # 0-10 s standby, 10-25 s battle, 25-30 s last of 2028
            mode = 0 if t < 10 else (1 if t < 25 else 2)
            roll = 150 * math.sin(now / 3) if mode == 2 else 30 * math.sin(now / 2)
            self.q.put(("status", decode_status(encode_status(mode, roll, 10 * math.sin(now / 5), (now * 20) % 360 - 180)), False))

            if mode == 0 and now - last_sensors >= 0.5:
                last_sensors = now
                humidity = 55 + 20 * math.sin(now / 4)   # crosses the 70 %RH limit now and then
                mask = 1 << 2 if humidity > 70 else 0
                self.q.put(("sensors", decode_sensors(encode_sensors(
                    31 + random.uniform(-0.2, 0.2), 1008.5 + random.uniform(-0.1, 0.1), humidity,
                    1.0 + random.uniform(-0.02, 0.02), random.uniform(0, 0.05), 0.45 + random.uniform(-0.01, 0.01),
                    mask)), False))

            if mode == 1 and now - last_press >= 1.5:   # same rules as chargeLaser() + shootLaser()
                last_press = now
                charge = min(charge + 3, 10)
                if charge >= 5:
                    charge -= 5
                    event = LASER_EVT_FIRED
                else:
                    event = LASER_EVT_NO_POWER
                self.q.put(("laser", decode_laser(encode_laser(charge, 10, 5, 3, event)), False))
            time.sleep(0.25)


# ---------------------------------------------------------------- window

BG = "#f4f5f7"
CARD = "#ffffff"
TEXT = "#1c1f24"
MUTED = "#8a9099"
ALERT_BG = "#fde2e2"
ALERT_TEXT = "#b3261e"
OK_GREEN = "#2e7d32"
CHARGING = "#e65100"
SEGMENT_ON = "#00897b"
SEGMENT_OFF = "#dde1e6"


class Dashboard:
    def __init__(self, root, source_q, worker):
        self.root = root
        self.q = source_q
        self.worker = worker
        self.mode = None
        self.sensors_time = None
        self.sensor_values = None
        self.event_clear_job = None

        root.title("EE2028 Drone Dashboard")
        root.configure(bg=BG)
        root.minsize(620, 560)
        root.protocol("WM_DELETE_WINDOW", self.close)

        # Connection line + mode badge
        top = tk.Frame(root, bg=BG)
        top.pack(fill="x", padx=16, pady=(14, 6))
        self.link_dot = tk.Label(top, text="●", fg=MUTED, bg=BG, font=("Helvetica", 14))
        self.link_dot.pack(side="left")
        self.link_label = tk.Label(top, text="Starting…", fg=TEXT, bg=BG, font=("Helvetica", 11), anchor="w")
        self.link_label.pack(side="left", padx=(4, 0), fill="x", expand=True)

        self.mode_label = tk.Label(root, text="NOT CONNECTED", fg="white", bg=MUTED,
                                   font=("Helvetica", 22, "bold"), pady=8)
        self.mode_label.pack(fill="x", padx=16, pady=(0, 10))

        # Sensor tiles
        sensors_card = self._card(root, "Sensors")
        self.sensor_note = tk.Label(sensors_card, text="Waiting for data…", fg=MUTED, bg=CARD,
                                    font=("Helvetica", 9), anchor="w")
        self.sensor_note.grid(row=0, column=0, columnspan=3, sticky="w", padx=10)
        self.tiles = {}
        for i, (key, label, unit, _fmt, _bit) in enumerate(SENSORS):
            self.tiles[key] = self._sensor_tile(sensors_card, label, unit, row=1 + i // 3, col=i % 3)
        for c in range(3):
            sensors_card.grid_columnconfigure(c, weight=1, uniform="tiles")

        # Orientation
        orient_card = self._card(root, "Orientation")
        self.orient = {}
        for i, axis in enumerate(("Roll", "Pitch", "Yaw")):
            cell = tk.Frame(orient_card, bg=CARD)
            cell.grid(row=0, column=i, sticky="nsew", padx=10, pady=(0, 10))
            tk.Label(cell, text=axis, fg=MUTED, bg=CARD, font=("Helvetica", 10)).pack(anchor="w")
            value = tk.Label(cell, text="—", fg=TEXT, bg=CARD, font=("Helvetica", 18, "bold"))
            value.pack(anchor="w")
            self.orient[axis.lower()] = value
            orient_card.grid_columnconfigure(i, weight=1, uniform="orient")

        # Laser
        laser_card = self._card(root, "Laser")
        self.laser_canvas = tk.Canvas(laser_card, height=46, bg=CARD, highlightthickness=0)
        self.laser_canvas.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10)
        self.laser_canvas.bind("<Configure>", lambda _e: self._draw_laser())
        self.laser_text = tk.Label(laser_card, text="—", fg=TEXT, bg=CARD, font=("Helvetica", 14, "bold"))
        self.laser_text.grid(row=1, column=0, sticky="w", padx=10, pady=(4, 10))
        self.laser_event = tk.Label(laser_card, text="", fg=TEXT, bg=CARD, font=("Helvetica", 12, "bold"), padx=8)
        self.laser_event.grid(row=1, column=1, sticky="e", padx=10, pady=(4, 10))
        laser_card.grid_columnconfigure(0, weight=1)
        self.laser = None

        self.root.after(50, self._poll)
        self._schedule_refresh()

    # -- layout helpers
    def _card(self, parent, title):
        outer = tk.Frame(parent, bg=CARD, highlightbackground="#d9dce1", highlightthickness=1)
        outer.pack(fill="x", padx=16, pady=6)
        tk.Label(outer, text=title, fg=TEXT, bg=CARD, font=("Helvetica", 12, "bold")).pack(anchor="w", padx=10, pady=(8, 2))
        body = tk.Frame(outer, bg=CARD)
        body.pack(fill="x")
        return body

    def _sensor_tile(self, parent, label, unit, row, col):
        tile = tk.Frame(parent, bg=CARD, padx=10, pady=8)
        tile.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
        name = tk.Label(tile, text=label, fg=MUTED, bg=CARD, font=("Helvetica", 10), anchor="w")
        name.pack(anchor="w")
        value = tk.Label(tile, text="—", fg=TEXT, bg=CARD, font=("Helvetica", 18, "bold"), anchor="w")
        value.pack(anchor="w")
        unit_label = tk.Label(tile, text=unit, fg=MUTED, bg=CARD, font=("Helvetica", 9), anchor="w")
        unit_label.pack(anchor="w")
        return {"frame": tile, "name": name, "value": value, "unit": unit_label, "unit_text": unit}

    # -- message handling
    def _poll(self):
        try:
            while True:
                msg = self.q.get_nowait()
                kind = msg[0]
                if kind == "link":
                    self._show_link(msg[1], msg[2])
                elif kind == "status":
                    self._show_status(msg[1])
                elif kind == "sensors":
                    self.sensor_values = msg[1]
                    self.sensors_time = time.monotonic()
                    self._refresh_sensor_state()
                elif kind == "laser":
                    self._show_laser(msg[1], initial=msg[2])
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    def _show_link(self, text, connected):
        self.link_label.config(text=text)
        self.link_dot.config(fg=OK_GREEN if connected else MUTED)
        if not connected and self.mode != MODE_DESTROYED:   # DESTROYED stays: it is why the link dropped
            self.mode = None   # don't keep showing a mode we can no longer see
            self.mode_label.config(text="NOT CONNECTED", bg=MUTED)

    def _show_status(self, status):
        mode = status["mode"]
        if mode != self.mode:
            self.mode = mode
            label, colour = MODES.get(mode, (f"MODE {mode}", MUTED))
            self.mode_label.config(text=label, bg=colour)
            self._refresh_sensor_state()
        for axis in ("roll", "pitch", "yaw"):
            self.orient[axis].config(text=f"{status[axis]:+.1f}°")

    def _refresh_sensor_state(self):
        values = self.sensor_values
        age = time.monotonic() - self.sensors_time if self.sensors_time else None
        in_standby = self.mode in (None, 0)
        stale = values is None or not in_standby or (age is not None and age > SENSOR_STALE_S)

        if values is None:
            note = "Waiting for data (sensors are sent in standby only)…"
        elif not in_standby:
            note = "Sensors are sent in standby only. Showing the last standby readings."
        elif stale:
            note = f"No sensor update for {age:.0f} s"
        else:
            note = "Live · sent every 0.5 s in standby · red = out of range"
        self.sensor_note.config(text=note)

        for key, _label, unit, fmt, bit in SENSORS:
            tile = self.tiles[key]
            if values is None:
                text, alert = "—", False
            else:
                text, alert = fmt.format(values[key]), bool(values["mask"] & (1 << bit))
            bg = ALERT_BG if alert and not stale else CARD
            fg = MUTED if stale else (ALERT_TEXT if alert else TEXT)
            for widget in (tile["frame"], tile["name"], tile["value"], tile["unit"]):
                widget.config(bg=bg)
            tile["value"].config(text=text, fg=fg)
            tile["unit"].config(text=f"{unit} · OUT OF RANGE" if alert and not stale else unit,
                                fg=ALERT_TEXT if alert and not stale else MUTED)

    def _schedule_refresh(self):
        """Re-checks every 0.5 s so tiles grey out when updates stop."""
        self._refresh_sensor_state()
        self.root.after(500, self._schedule_refresh)

    def _show_laser(self, laser, initial):
        self.laser = laser
        self._draw_laser()
        charge, max_charge, cost = laser["charge"], laser["max"], laser["cost"]
        # Each press first adds charge, then fires if there is enough (chargeLaser + shootLaser)
        if min(charge + laser["per_press"], max_charge) >= cost:
            self.laser_text.config(text=f"{charge} / {max_charge}  ·  NEXT PRESS FIRES", fg=OK_GREEN)
        else:
            self.laser_text.config(text=f"{charge} / {max_charge}  ·  NEXT PRESS CHARGES", fg=CHARGING)

        if initial or laser["event"] == LASER_EVT_NONE:
            return
        if laser["event"] == LASER_EVT_FIRED:
            self.laser_event.config(text="Laser fired!", fg="white", bg=ALERT_TEXT)
        else:
            self.laser_event.config(text="Insufficient power!", fg=CHARGING, bg="#fff3e0")
        if self.event_clear_job:
            self.root.after_cancel(self.event_clear_job)
        self.event_clear_job = self.root.after(1500, lambda: self.laser_event.config(text="", bg=CARD))

    def _draw_laser(self):
        c = self.laser_canvas
        c.delete("all")
        if not self.laser:
            return
        width = max(c.winfo_width(), 100)
        max_charge = max(self.laser["max"], 1)
        gap, top, bottom = 4, 6, 32
        seg_w = (width - gap * (max_charge - 1)) / max_charge
        for i in range(max_charge):
            x0 = i * (seg_w + gap)
            colour = SEGMENT_ON if i < self.laser["charge"] else SEGMENT_OFF
            c.create_rectangle(x0, top, x0 + seg_w, bottom, fill=colour, outline="")
        # Marker: charge needed for one shot
        cost = self.laser["cost"]
        if 0 < cost <= max_charge:
            x = cost * (seg_w + gap) - gap / 2
            c.create_line(x, 0, x, bottom + 4, fill=TEXT, width=2)
            c.create_text(x, bottom + 9, text=f"shot cost ({cost})", fill=MUTED, font=("Helvetica", 8))

    def close(self):
        self.worker.stop()
        self.worker.join(timeout=3)   # lets the BLE worker disconnect cleanly
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Live Bluetooth dashboard for the EE2028 drone board")
    parser.add_argument("--name", default=DEVICE_NAME, help=f"advertised name to look for (default {DEVICE_NAME})")
    parser.add_argument("--address", help="connect to this Bluetooth address instead of scanning by name")
    parser.add_argument("--demo", action="store_true", help="show fake data, no board needed")
    args = parser.parse_args()

    messages = queue.Queue()
    worker = DemoWorker(messages) if args.demo else BleWorker(messages, args.name, args.address)

    root = tk.Tk()
    Dashboard(root, messages, worker)
    worker.start()
    root.mainloop()


if __name__ == "__main__":
    main()
