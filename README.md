SIUUUUUUUUUUUU
Jack is Gay

## Bluetooth dashboard

The board advertises over Bluetooth LE as `EE2028-Drone` and sends:
- sensor readings (T, P, H, accel, gyro, mag) every 0.5 s in standby,
- mode and roll/pitch/yaw every 0.25 s in every mode,
- the laser charge after every laser action.

When the drone is destroyed it sends a last DESTROYED status, disconnects and switches
Bluetooth off. Reset the board to turn it back on.

The packet layout is documented in `Core/Inc/ble_drone.h`.

Set up the laptop app once (Linux, macOS or Windows; Python 3.9+ with tkinter):

```
python3 -m venv --system-site-packages host/.venv
host/.venv/bin/pip install -r host/requirements.txt
```

Run it while the board is running (no pairing needed):

```
host/.venv/bin/python host/drone_dashboard.py            # scans for EE2028-Drone
host/.venv/bin/python host/drone_dashboard.py --demo     # fake data, no board needed
```

The board prints its Bluetooth address on the UART at startup; pass `--address` to skip the scan.
