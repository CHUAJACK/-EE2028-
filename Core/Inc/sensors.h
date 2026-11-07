/**
 * @file    sensors.h
 * @brief   Sensor reading, IMU filtering and threshold checking.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "mahony.h"

#define MDPS_TO_RADS (3.1415926535f/180000.0f)   //milli-degrees/s to rad/s
#define GYRO_CALIBRATE_COUNT 20

//setting the thresholds
//temperature in degC: safe operating range for Li-ion battery and electronics
#define TEMP_LOW       0.0f
#define TEMP_HIGH      45.0f
//humidity in %RH: comfortable range for humans
#define HUMIDITY_LOW   30.0f
#define HUMIDITY_HIGH  70.0f
//acceleration in g: below 0.5*9.8 = free fall, above 2*9.8 = collision
#define ACCEL_LOW      0.5f
#define ACCEL_HIGH     2.0f
//rotation in rads-1: above 250 dps = drone is tumbling
#define GYRO_LOW       (0.0f * MDPS_TO_RADS)
#define GYRO_HIGH      (250000.0f * MDPS_TO_RADS)
//magnetic field in gauss: Earth's field is about 0.25 to 0.65 gauss
#define MAGNETO_LOW    0.20f
#define MAGNETO_HIGH   2.50f


//for pressure, the pressure difference relative to takeoff is calculated (range is -10m to 120m) because pressure differs too much 
//pressure drops about 0.12 hPa for every metre the drone climbs, so
#define PRESSURE_DROP_FOR_MAX_HEIGHT  14.4f
#define PRESSURE_RISE_FOR_MIN_HEIGHT  1.2f

extern float pressureLow;
extern float pressureHigh;

extern int sensorOutOfRange[6];     //1 = out of threshold, 0 = within threshold
extern char *sensorLabels[6];

extern float roll;                  //latest orientation from the Mahony filter, in degrees
extern float pitch;
extern float yaw;

int checkOneSensor(char *name, float reading, float low, float high, char *unit);
int determineSensorValidity(float temperature, float pressure, float humidity, float accel, float gyro, float magneto);
uint8_t sensorViolationMask(void);
float measurePressure(void);
void readImuMagnetometer(float accelG[3], float magnetoDataFloat[3], float gyroDataRadS[3]);
void processMahonyFilter(mahony_t* mahonyFilter, float accelG[3], float magnetoDataFloat[3], float gyroDataRadS[3]);
void readTPHSensorData(float* tempData, float* pressureData, float* humidityData);

#endif /* SENSORS_H */
