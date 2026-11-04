/**
  ******************************************************************************
  * @file    ble_drone.h
  * @brief   Bluetooth LE telemetry for the drone (SPBTLE-RF / BlueNRG-MS).
  *
  * The board advertises as BLE_DEVICE_NAME and exposes one GATT service with
  * three Read + Notify characteristics (all little-endian):
  *
  *   Service  e2028000-6c61-7365-7200-647230ee2028
  *   Sensors  e2028001-...  13 bytes, sent in standby only
  *              int16  temperature  x100 (degC)
  *              uint16 pressure     x10  (hPa)
  *              uint16 humidity     x100 (%RH)
  *              uint16 |accel|      x1000 (g)
  *              uint16 |gyro|       x1000 (rad/s)
  *              uint16 |magneto|    x1000 (gauss)
  *              uint8  out-of-range bitmask (bit0 T, 1 P, 2 H, 3 A, 4 G, 5 M)
  *   Status   e2028002-...  7 bytes, sent in every mode. When the drone is
  *                          destroyed it is sent once more, then BLE switches off.
  *              uint8  drone mode (0 standby, 1 battle, 2 last of 2028, 100 destroyed)
  *              int16  roll, pitch, yaw x100 (deg)
  *   Laser    e2028003-...  5 bytes, sent after every laser action
  *              uint8  charge, max charge, cost per shot, charge added per press,
  *                     last event (BLE_LASER_EVT_*)
  ******************************************************************************
  */
#ifndef BLE_DRONE_H
#define BLE_DRONE_H

#include <stdint.h>

#define BLE_DEVICE_NAME          "EE2028-Drone"

#define BLE_LASER_EVT_NONE       0
#define BLE_LASER_EVT_FIRED      1
#define BLE_LASER_EVT_NO_POWER   2

/* Starts the BLE module and the GATT service.
   Returns 0 on success, otherwise the number of the step that failed.
   On failure all other BLE_ functions do nothing, so the drone keeps working. */
int  BLE_Init(void);

/* Call every main-loop pass: handles connect/disconnect and re-advertising. */
void BLE_Process(void);

/* Call from HAL_GPIO_EXTI_Callback for the BlueNRG IRQ pin (PE6). */
void BLE_IrqHandler(void);

/* Switches BLE off (used when the drone is destroyed): lets queued updates go
   out, disconnects the laptop and holds the module in reset. After this all
   BLE_ functions do nothing. */
void BLE_Shutdown(void);

int  BLE_IsConnected(void);

/* Writes the board's BLE address as "XX:XX:XX:XX:XX:XX" (needs 18 bytes). */
void BLE_GetAddress(char *out);

void BLE_UpdateSensors(float tempC, float pressureHpa, float humidityRh,
                       float accelG, float gyroRadS, float magGauss,
                       uint8_t outOfRangeMask);
void BLE_UpdateStatus(uint8_t mode, float rollDeg, float pitchDeg, float yawDeg);
void BLE_UpdateLaser(uint8_t charge, uint8_t maxCharge, uint8_t cost, uint8_t chargePerPress,
                     uint8_t event);

#endif /* BLE_DRONE_H */
