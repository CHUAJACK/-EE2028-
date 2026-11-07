# Bluetooth architecture

The drone streams its telemetry to a laptop over Bluetooth Low Energy (BLE). The board is
the **peripheral** (it advertises and the laptop connects to it); the laptop runs
`host/drone_dashboard.py` as the **central**. Nothing is ever sent from the laptop to the
board: the link is one-way telemetry, and no pairing is needed.

This document follows the data from the sensors to the dashboard window, layer by layer.

- [1. Big picture](#1-big-picture)
- [2. Hardware and SPI transport](#2-hardware-and-spi-transport)
- [3. Firmware layers](#3-firmware-layers)
- [4. The lifecycle of the radio](#4-the-lifecycle-of-the-radio)
- [5. GATT: what the board exposes](#5-gatt-what-the-board-exposes)
- [6. Packet formats](#6-packet-formats)
- [7. When the main loop sends what](#7-when-the-main-loop-sends-what)
- [8. Concurrency: interrupt vs main loop](#8-concurrency-interrupt-vs-main-loop)
- [9. Failure handling](#9-failure-handling)
- [10. The laptop side](#10-the-laptop-side)
- [11. Known issues and gotchas](#11-known-issues-and-gotchas)
- [12. How to change things](#12-how-to-change-things)

---

## 1. Big picture

```
 B-L4S5I-IOT01A board                                                   Laptop
┌──────────────────────────────────────────────────────┐         ┌────────────────────────┐
│  main.c  (main loop)                                 │         │  drone_dashboard.py    │
│    │  BLE_Process / BLE_Update*(...)                 │         │   tkinter window       │
│    ▼                                                 │         │        ▲ queue         │
│  ble_drone.c      GATT service, advertising,         │         │   BleWorker thread     │
│    │              connect/disconnect, packet packing │         │   (asyncio + bleak)    │
│    ▼                                                 │         │        ▲               │
│  ST BlueNRG-MS stack (Middlewares/ST/BlueNRG-MS)     │         │   OS Bluetooth stack   │
│    │  HCI commands / events                          │         └───────────▲────────────┘
│    ▼                                                 │                     │
│  hci_tl_interface.c   SPI3 transport + IRQ pin       │                     │ BLE radio
│    │  SPI3 + CS + RST + IRQ                          │                     │ (2.4 GHz)
│    ▼                                                 │                     │
│  SPBTLE-RF module (BlueNRG-MS chip) ─────────────────┼─────────────────────┘
└──────────────────────────────────────────────────────┘
```

Key ideas:

1. **The BLE stack does not run on the STM32.** The SPBTLE-RF module has its own
   processor (BlueNRG-MS) that runs the link layer, GAP and GATT. The STM32 only sends it
   HCI commands over SPI and receives HCI events back.
2. **The STM32 pushes data.** After each update the module sends a *notification* to the
   laptop, if the laptop has subscribed.
3. **Bluetooth must never hurt the drone.** If the module fails to start, stops answering,
   or the laptop disconnects, the flight code carries on. Every `BLE_*` function becomes a
   no-op when BLE is off.

### Source files

| File | Role |
|---|---|
| `Core/Inc/ble_drone.h`, `Core/Src/ble_drone.c` | The application API and GATT service. This is the only BLE file `main.c` talks to. |
| `Core/Src/hci_tl_interface.c`, `Core/Inc/hci_tl_interface.h` | SPI3 transport between the STM32 and the module, pin map, interrupt handling. |
| `Core/Inc/bluenrg_conf.h` | Configuration for the ST library. |
| `Middlewares/ST/BlueNRG-MS/` | ST's BlueNRG-MS stack (HCI layer, GAP/GATT command wrappers, packet queues). Unmodified ST code. |
| `Core/Src/stm32l4xx_it.c` | `EXTI9_5_IRQHandler` forwards pin 6 (the module's IRQ line) to HAL. |
| `Core/Src/main.c` | Calls `BLE_Init`, `BLE_Process`, `BLE_Update*`, `BLE_Shutdown`; forwards the EXTI callback. |
| `host/drone_dashboard.py` | Laptop-side dashboard (Python, `bleak` + `tkinter`). |

---

## 2. Hardware and SPI transport

The module is wired to **SPI3** (shared with the Wi-Fi module and the sub-GHz radio).

| Signal | STM32 pin | Notes |
|---|---|---|
| SCK / MISO / MOSI | PC10 / PC11 / PC12 | SPI3, alternate function AF6 |
| CS (chip select) | PD13 | Active low, idles high |
| RESET | PA8 | Held low = module in reset |
| IRQ | PE6 | **Rising edge = the module has data for us.** EXTI line 6, pull-down |
| Status LED | PC9 | LED4 (blue) = BLE connected. Shares the pin with the Wi-Fi LED3 |

SPI settings (`HCI_TL_SPI_Init`): master, mode 0 (CPOL=0, CPHA=0), 8-bit, MSB first,
software NSS, prescaler /2 (about 2 MHz with the 4 MHz MSI clock).

Because SPI3 is shared, init drives the Wi-Fi chip select (PE0) and the sub-GHz radio chip
select (PB5) high so those devices ignore the bus.

### The BlueNRG SPI protocol

Every transfer begins with a 5-byte header. The module answers in the same transfer:

```
Master sends:  [ 0x0A (write) or 0x0B (read) | 0 | 0 | 0 | 0 ]
Module returns:[ 0x02 (READY)  | write-buffer space | 0 | read-byte-count lo | hi ]
```

- **Write** (`HCI_TL_SPI_Send`, runs in the main loop): send the header. If the module
  answers READY and has enough buffer space (`header_slave[1] >= size`), clock out the HCI
  command. Otherwise release CS and retry, up to `TIMEOUT_DURATION` (15 ms), then give up
  with -3. The IRQ interrupt is **disabled** during a write so a receive cannot interleave
  on the same bus; an IRQ edge that arrives meanwhile stays pending and fires on re-enable.
- **Read** (`HCI_TL_SPI_Receive`, runs in the EXTI interrupt): send the read header, take
  the byte count from the reply, then clock out `0xFF` dummy bytes while capturing the
  packet. The count is clamped to the caller's buffer so a bad header can't overrun it.

### Reset

`HCI_TL_SPI_Reset` pulls RESET low for 5 ms, then high for 5 ms, with CS held high so the
module doesn't see a spurious command while booting.

---

## 3. Firmware layers

From the top down:

### 3.1 `ble_drone.c`: the application layer

This is the module you edit. It owns:

- the state flags (`ble_ready`, `connected`, `set_connectable`),
- the GATT service and its three characteristics,
- advertising and re-advertising,
- packing sensor values into byte arrays,
- the shutdown logic.

Public API (`ble_drone.h`):

| Function | Called from | Purpose |
|---|---|---|
| `BLE_Init()` | `main()` once, after the pressure baseline | Bring up the module and GATT. Returns 0 or the failing step (1-8). |
| `BLE_Process()` | every main-loop pass | Service pending events; restart advertising when needed. |
| `BLE_IrqHandler()` | `HAL_GPIO_EXTI_Callback` (interrupt) | Pull a packet from the module into a queue. |
| `BLE_UpdateSensors(...)` | main loop, standby, once per second | Publish sensor readings. |
| `BLE_UpdateStatus(...)` | main loop, every 250 ms | Publish mode + roll/pitch/yaw. |
| `BLE_UpdateLaser(...)` | main loop, on laser actions | Publish laser charge and last event. |
| `BLE_Shutdown()` | main loop, when the drone is destroyed | Flush, disconnect, power the radio down. |
| `BLE_IsConnected()`, `BLE_GetAddress()` | startup / diagnostics | Connection flag; address string for the UART log. |

### 3.2 ST BlueNRG-MS stack (`Middlewares/ST/BlueNRG-MS`)

ST's library. It turns calls like `aci_gatt_add_char(...)` into HCI command bytes, sends
them through the transport and (for synchronous commands) waits for the matching
completion event. It also owns the **packet queues**:

- `hciReadPacketBuffer[5]`: a fixed pool of 5 receive buffers.
- `hciReadPktPool`: free buffers. `hciReadPktRxQueue`: received, not yet processed.

Two functions matter:

- `hci_notify_asynch_evt()` (interrupt side): takes a free buffer from the pool, calls the
  transport's `Receive`, and puts the packet on the RX queue.
- `hci_user_evt_proc()` (main-loop side): takes packets off the RX queue and invokes the
  application's event callback (`ble_event_handler`), then returns the buffer to the pool.

`hci_send_req()` sends a command and then busy-waits (polling the RX queue) for the
response, with a timeout of about 1 s (`HCI_DEFAULT_TIMEOUT_MS`). This is why a dead
module can freeze the main loop (see [section 9](#9-failure-handling)).

### 3.3 `hci_tl_interface.c`: the transport

Registers the SPI functions with the stack (`hci_tl_lowlevel_init` fills a `tHciIO` struct
with `Init`, `DeInit`, `Send`, `Receive`, `Reset`, `GetTick`). It also implements the
interrupt-side loop `hci_tl_lowlevel_isr`:

```c
while (IsDataAvailable() && reads++ < ISR_MAX_READS) {      // IRQ pin still high?
    if (hci_notify_asynch_evt(NULL)) return;                // no free buffer: stop
}
```

The module keeps its IRQ pin high while it has events queued, so the loop drains them
(capped at 20 per interrupt as a safety limit).

---

## 4. The lifecycle of the radio

```
 power on
    │
    ▼
 BLE_Init() ──fail──► ble_shutdown(): radio off, drone keeps flying, returns step 1-8
    │ ok
    ▼
 advertising ◄──────────────────────────────┐
    │ laptop connects                       │ EVT_DISCONN_COMPLETE
    ▼                                       │ (set_connectable = 1)
 connected  (LED4 on) ──────────────────────┘
    │
    │ drone destroyed
    ▼
 BLE_Shutdown(): flush ► disconnect ► reset line held low ► permanently off
```

### 4.1 `BLE_Init()` step by step

| Step | Call | Failure code |
|---|---|---|
| 0 | `hci_init(ble_event_handler, NULL)`: sets up SPI3, resets the module | |
| 1 | `getBlueNRGVersion`: first command after reset, **retried once** after 20 ms in case the module is still booting | 1 |
| 2 | `aci_hal_read_config_data`: reads the board's static random address (stored for `BLE_GetAddress`); a failure here is non-fatal | |
| 3 | `aci_gatt_init` | 2 |
| 4 | `aci_gap_init_IDB05A1` as peripheral; gets the GAP service handles | 3 |
| 5 | `aci_gatt_update_char_value` writes the device name `EE2028-Drone` into the GAP name characteristic | 4 |
| 6 | `aci_gatt_add_serv`: one primary service, 10 attribute records (1 + 3 per characteristic × 3) | 5 |
| 7 | `add_char` for Sensors, Status, Laser | 6 / 7 / 8 |
| 8 | `aci_hal_set_tx_power_level(1, 4)`: high-power mode, level 4 | |
| 9 | set `ble_ready`, request advertising, call `BLE_Process()` once | |

On any failure `ble_shutdown()` runs and the code is returned. `main.c` prints
`Bluetooth init failed at step N, continuing without BLE` and carries on.

### 4.2 Advertising

`start_advertising()` makes the board a connectable, undirected advertiser (`ADV_IND`):

- Interval 100-200 ms (`ADV_INTERVAL_MIN/MAX` = 160/320 × 0.625 ms).
- Address type: static random (the module's own address, different per board).
- **Advertising data** carries the complete local name `EE2028-Drone`.
- **Scan response** carries the 128-bit service UUID. It does not fit beside the name in
  the 31-byte advertising packet, so it goes in the scan response. The laptop matches on
  either the name or the UUID.

`BLE_Process()` calls `start_advertising()` whenever `set_connectable` is set. If the
command fails it retries 1 s later (`ADV_RETRY_MS`) instead of hammering the module. The
time comparison `(int32_t)(HAL_GetTick() - adv_retry_at) >= 0` is wrap-around safe.

### 4.3 Connection events

`ble_event_handler` (called from `hci_user_evt_proc` in the main loop) handles two events:

- `EVT_LE_CONN_COMPLETE`: on success, `connected = 1`, remember `conn_handle`, LED4 on. On
  failure, ask to advertise again.
- `EVT_DISCONN_COMPLETE`: `connected = 0`, LED4 off, `set_connectable = 1` so the board
  becomes visible again and the laptop can reconnect.

The module stops advertising by itself when a connection is made, so there's no explicit
stop.

### 4.4 `BLE_Shutdown()`

Used when the drone self-destructs:

1. If connected, keep servicing events for 250 ms (`FLUSH_MS`) so queued updates leave the
   module.
2. Send `aci_gap_terminate` and wait up to 500 ms (`DISCONNECT_WAIT_MS`) for the
   disconnect event. The laptop sees the drop immediately instead of waiting for a link
   timeout.
3. `ble_shutdown()`: disable the IRQ, hold RESET low (the radio is off and not
   advertising), clear all flags.

After this every `BLE_*` function returns immediately. **Only a board reset turns
Bluetooth back on.**

---

## 5. GATT: what the board exposes

One custom primary service with three characteristics. All 128-bit UUIDs share the base
`e20280xx-6c61-7365-7200-647230ee2028`, where `xx` is the ID:

| ID | UUID | Name | Length | Properties |
|---|---|---|---|---|
| `00` | `e2028000-6c61-7365-7200-647230ee2028` | Service | n/a | Primary service |
| `01` | `e2028001-6c61-7365-7200-647230ee2028` | Sensors | 13 bytes | Read, Notify |
| `02` | `e2028002-6c61-7365-7200-647230ee2028` | Status | 7 bytes | Read, Notify |
| `03` | `e2028003-6c61-7365-7200-647230ee2028` | Laser | 5 bytes | Read, Notify |

The ASCII bytes of the base spell out `…6c 61 73 65 72 = "laser"`, `…64 72 30 = "dr0"`.

- **Read** returns the last value written (`GATT_DONT_NOTIFY_EVENTS` means the stack
  answers reads itself, with no event to the STM32). That's how a freshly connected
  laptop gets the current state immediately.
- **Notify** pushes every later update. The laptop must first write the characteristic's
  *Client Characteristic Configuration Descriptor* to subscribe. `bleak.start_notify`
  does that. This is why each characteristic reserves 3 attribute records (declaration,
  value, notify config).
- **Byte order:** all multi-byte numbers are **little-endian**. The 16-byte UUID is stored
  LSB-first for the BlueNRG stack, which is what `make_uuid()` builds (it reverses the
  MSB-first bytes and then patches byte 12 with the ID).

### How a value reaches the laptop

```
 main loop:   BLE_UpdateStatus(...)
                 │ pack 7 bytes
                 ▼
              update_char() ─► aci_gatt_update_char_value()   (blocking HCI command over SPI)
                                      │
                                      ▼
                      module stores the value, and if the laptop has enabled
                      notifications, transmits an ATT notification over the air
                                      │
                                      ▼
              laptop (bleak):  callback(char, data) ─► decode ─► queue ─► tkinter
```

---

## 6. Packet formats

Scaling is applied on the board (`scale_i16` / `scale_u16`) so values travel as integers.
Both helpers round to nearest, clamp to the type's range and turn NaN into 0, so a bad
sensor reading cannot produce garbage.

### Sensors (13 bytes, standby only)

| Offset | Type | Field | Scale | Unit |
|---|---|---|---|---|
| 0 | int16 | temperature | ×100 | °C |
| 2 | uint16 | pressure | ×10 | hPa |
| 4 | uint16 | humidity | ×100 | %RH |
| 6 | uint16 | \|acceleration\| | ×1000 | g |
| 8 | uint16 | \|gyro\| | ×1000 | rad/s |
| 10 | uint16 | \|magnetometer\| | ×1000 | gauss |
| 12 | uint8 | out-of-range bitmask | | bit0 T, 1 P, 2 H, 3 A, 4 G, 5 M |

The mask comes from `sensorViolationMask()` in `sensors.c`, the same threshold check that
drives the OLED boxes and the UART messages. The three motion/field values are vector
magnitudes, not per-axis.

Python format string: `"<hHHHHHB"`.

### Status (7 bytes, every mode)

| Offset | Type | Field | Scale |
|---|---|---|---|
| 0 | uint8 | drone mode | 0 standby, 1 battle, 2 last of 2028, 100 destroyed |
| 1 | int16 | roll | ×100 (deg) |
| 3 | int16 | pitch | ×100 (deg) |
| 5 | int16 | yaw | ×100 (deg) |

Python format string: `"<Bhhh"`.

### Laser (5 bytes, after every laser action and once at startup)

| Offset | Type | Field |
|---|---|---|
| 0 | uint8 | current charge |
| 1 | uint8 | max charge (`LASER_MAX`, 10) |
| 2 | uint8 | cost per shot (`LASER_COST`, 5) |
| 3 | uint8 | charge added per press (`LASER_CHARGE_PER_PRESS`, 3) |
| 4 | uint8 | last event: 0 none, 1 fired, 2 no power (`BLE_LASER_EVT_*`) |

Python format string: `"<BBBBB"`. Max, cost and per-press are sent with every packet, so
the dashboard never hard-codes them: changing the `#define`s in firmware is enough.

> The packet layout is defined twice: in `ble_drone.h` / `ble_drone.c` and in
> `host/drone_dashboard.py`. They must be kept in sync by hand.

---

## 7. When the main loop sends what

Everything is sent from the single main loop (`main.c`), in this order each pass:

```
 while (1) {
     tickStart = HAL_GetTick();
     ...display / IMU / button / LED work...
     BLE_Process();                                       // service events, re-advertise
     if (tickStart - lastBleStatusTick >= 250) {          // every 250 ms, any mode
         BLE_UpdateStatus(mode, roll, pitch, yaw);
     }
     standby:  every 1000 ms  →  BLE_UpdateSensors(...)
     battle:   on button press →  BLE_UpdateLaser(...)
 }
```

| Data | Trigger | Rate |
|---|---|---|
| Status | timer (`BLE_STATUS_PERIOD_MS` = 250) | 4 Hz, all modes |
| Sensors | sensor-read timer in standby | 1 Hz, standby only |
| Laser | startup, then each charge/shoot action in battle | event-driven |
| Final status + shutdown | destroyed (10 s countdown expires) | once |

The laser characteristic is also sent once at startup (`main.c`) so a laptop that connects
before the first shot reads sensible values.

---

## 8. Concurrency: interrupt vs main loop

There is no RTOS. Two contexts share the SPI bus and the HCI queues:

| Context | What runs there |
|---|---|
| **EXTI interrupt** (priority 5) | `BLE_IrqHandler` → `hci_tl_lowlevel_isr` → SPI **read** → push packet on RX queue |
| **Main loop** | `hci_user_evt_proc` (pop queue, run callbacks), every SPI **write** (commands), all `BLE_Update*` |

The path of an incoming event: PE6 rising edge → `EXTI9_5_IRQHandler` →
`HAL_GPIO_EXTI_IRQHandler(SPBTLE_RF_IRQ_EXTI6_Pin)` → `HAL_GPIO_EXTI_Callback` in
`main.c` → `BLE_IrqHandler()`. The same callback also handles the user button (PC13), so
it dispatches on the pin number.

Safeguards against the two contexts colliding:

1. **`HCI_TL_SPI_Send` disables the IRQ** while it holds the bus, so an interrupt cannot
   start a read in the middle of a write.
2. **A missed edge is recovered.** If the pool of 5 buffers is empty when the IRQ fires,
   the ISR returns without reading and the pin stays high. `poll_events()` in the main
   loop checks the pin after processing; if it is still high, it raises the interrupt
   again by software (`__HAL_GPIO_EXTI_GENERATE_SWIT`). Without this, BLE would stall
   forever after a single missed edge.
3. **Flags are `volatile`** (`ble_ready`, `connected`, `set_connectable`) because the
   interrupt and loop both touch them.

---

## 9. Failure handling

The design goal: BLE problems degrade telemetry only, never the flight logic.

| Situation | Behaviour |
|---|---|
| Module doesn't answer at boot | `BLE_Init` retries once, then returns step 1; BLE stays off; message on UART. |
| Any init step fails | Radio shut down, step number returned, drone runs normally. |
| Laptop disconnects | Event handler clears `connected`, LED off, board advertises again. |
| Advertising command fails | Retried every 1 s. |
| Notification fails because a TX buffer is full | That update is dropped. The next one carries fresh data. |
| Module stops responding (command timeout) | `update_char` counts consecutive `BLE_STATUS_TIMEOUT`s. After 3 (`MAX_TIMEOUTS`) BLE is switched off and `Bluetooth module not responding` is printed. This matters because each timeout is a busy-wait of about 1 s inside the main loop, which would otherwise freeze the IMU and button handling. |
| No laptop connected | `aci_gatt_update_char_value` still succeeds (it stores the value), so updates cost the same either way. |

Because the update calls are **synchronous HCI commands**, each one occupies the main
loop for the duration of an SPI round trip. At the current rates (4 Hz status, 1 Hz
sensors) that's negligible.

---

## 10. The laptop side

`host/drone_dashboard.py` (Python 3.9+, `bleak`, `tkinter`). See `README.md` for setup.

```
 ┌────────────────────┐  queue.Queue   ┌──────────────────────────┐
 │ BleWorker (thread) │ ─────────────► │ tkinter window (main     │
 │  asyncio + bleak   │  ("sensors",…) │ thread) polls the queue  │
 └────────────────────┘  ("status",…)  └──────────────────────────┘
                         ("laser",…)
                         ("link",text,connected)
```

tkinter is not thread-safe and asyncio wants its own thread, so the BLE worker never
touches widgets. It only puts tuples on a queue that the window drains.

`BleWorker._main()` loop:

1. **Find the board.** With `--address`, `find_device_by_address`. Otherwise
   `find_device_by_filter`, matching the name `EE2028-Drone` or the service UUID. 10 s
   timeout per scan, and the status line shows the attempt count so the user can tell it is
   still trying.
2. **Connect** with `BleakClient`, registering a `disconnected_callback`.
3. **Read once** each of the three characteristics (flagged `initial = True`), so the
   window fills in immediately instead of waiting for the next notification.
4. **Subscribe** with `start_notify` to all three. Each notification is decoded with
   `struct.unpack` and published. A wrong-length packet raises `struct.error` and is
   ignored.
5. **Wait** until the user closes the window or the link drops, then loop back to step 1
   after 2 s.

The decoders use the same format strings and scale factors as the firmware (section 6),
and the worker slices the data to the exact expected size so extra bytes don't break it.

`--demo` replaces `BleWorker` with `DemoWorker`, which generates fake packets **through the
same encode/decode functions**, so the demo exercises the real packet format without a
board.

The UI treats sensor data as stale after 3 s (`SENSOR_STALE_S`), which is expected in
battle mode because sensors aren't sent there.

---

## 11. Known issues and gotchas

1. **The DESTROYED status is never actually sent (bug).** In `main.c`, the "last update"
   is `BLE_UpdateStatus((uint8_t)droneMode, ...)` and happens **before**
   `droneMode = destroyed;`, so the final packet carries mode **2** (last of 2028), not
   **100**. The dashboard only learns the drone is destroyed from mode 100, so it shows
   *"Connection lost. Reconnecting…"* instead of the *"Drone destroyed"* message, and
   keeps scanning. Fix: set `droneMode = destroyed;` before the last `BLE_UpdateStatus` /
   `BLE_Shutdown()` calls (or pass `destroyed` explicitly).
2. **README says sensors go out every 0.5 s; the code does 1 s.** `main.c` gates the sensor
   block on `>= 1000` ms. The header comment, this document and the code agree on 1 s;
   only the README line is off.
3. **Packet layouts are duplicated** between firmware and `drone_dashboard.py`. Changing one
   without the other produces wrong numbers or dropped packets. There is no version byte.
4. **BLE is one-way.** The laptop cannot control the drone, and there is no write
   characteristic or authentication. Anyone in range can connect and read telemetry.
5. **Single connection.** The module serves one central at a time; while connected it
   isn't advertising, so a second laptop won't see it.
6. **Reset required after destruction.** `BLE_Shutdown` holds the module in reset on
   purpose; there is no runtime path to restart BLE.
7. **Blocking updates.** `BLE_Update*` block the main loop for one SPI exchange each. Don't
   add high-rate updates (hundreds of Hz) without measuring.

---

## 12. How to change things

### Add a new value to an existing packet

1. Increase the `*_LEN` define in `ble_drone.c` and update the layout comment in
   `ble_drone.h`.
2. Pack it in the matching `BLE_Update*` function using `put_le16`, `scale_i16` or
   `scale_u16`.
3. Update the `*_FMT`, `decode_*` and `encode_*` functions in `drone_dashboard.py`.
4. Rebuild and flash; restart the dashboard. (Reconnecting is enough: the characteristic
   length is fixed when the service is created at boot.)

### Add a new characteristic

1. Add `UUID_ID_*` and `*_LEN` defines and a `static uint16_t ...CharHandle`.
2. Call `add_char(...)` in `BLE_Init` after the existing ones and give it a new failure
   code.
3. **Raise the attribute count** in `aci_gatt_add_serv(..., 1 + 3 * 3, ...)`: it is
   `1 + 3 × number_of_characteristics`. Forgetting this makes `add_char` fail at the new
   characteristic.
4. Add a `BLE_UpdateXxx` function that calls `update_char`.
5. Add the UUID, decoder and an entry in `CHARACTERISTICS` on the laptop side.

### Change the update rates

- Status: `BLE_STATUS_PERIOD_MS` in `main.c`.
- Sensors: the `>= 1000` condition on `lastSensorTimeTick` in `main.c`. This is also the
  rate at which threshold messages go out over UART.
- Advertising interval: `ADV_INTERVAL_MIN` / `ADV_INTERVAL_MAX` in `ble_drone.c`.

### Debugging checklist

| Symptom | Check |
|---|---|
| `Bluetooth init failed at step 1` | Module not answering: check the SPI3 pins, that RESET (PA8) toggles, and that the Wi-Fi/sub-GHz chip selects are high. |
| Init fails at step 5 or 6 or 7/8 | Attribute budget in `aci_gatt_add_serv` too small, or a GATT call failing. |
| Board not found by the dashboard | LED4 off? The board isn't advertising. Is something else already connected to it? Try `--address` using the address the board prints at startup. |
| Connects but no data | Notifications not subscribed (client didn't write the CCCD), or the board is in a mode that doesn't send that characteristic (sensors only in standby). |
| Data stops after a while | `Bluetooth module not responding, BLE switched off` on the debug console: three timeouts in a row. |
| Everything stalls after a burst of traffic | IRQ edge recovery in `poll_events()` is not running, so check `BLE_Process()` is being called every pass. |
