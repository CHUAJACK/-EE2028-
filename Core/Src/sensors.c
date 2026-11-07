/**
 * @file    sensors.c
 * @brief   Sensor reading, IMU filtering and threshold checking.
 */
#include "sensors.h"
#include "main.h"
#include <stdio.h>
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"

float pressureLow;
float pressureHigh;

float roll;
float pitch;
float yaw;

//checks one sensor reading against its low and high limit
//prints a message and returns 1 if the reading is out of range, otherwise returns 0
int checkOneSensor(char *name, float reading, float low, float high, char *unit)
{
	char messageToUART[100];
	if (reading > high) {
//		printf("%s: %.2f (%s), exceeds threshold of %.2f (%s).\r\n", name, reading, unit, high, unit);
		sprintf(messageToUART, "%s: %.2f (%s), exceeds threshold of %.2f (%s).\r\n",name, reading, unit, high, unit);}
	else if (reading < low) {
//		printf("%s: %.2f (%s), is below threshold of %.2f (%s).\r\n", name, reading, unit, low, unit);
		sprintf(messageToUART, "%s: %.2f (%s), is below threshold of %.2f (%s).\r\n",name, reading, unit, low, unit);
	}else{
		return 0;
	}
	sendUART(messageToUART);
	return 1;
}

//checks all six sensors
int sensorOutOfRange[6] = {0};   //1 = out of threshold, 0 = within threshold
char *sensorLabels[6] = {"TEMP", "PRES", "HUM", "ACC", "GYRO", "MAG"};
int determineSensorValidity(float temperature, float pressure, float humidity, float accel, float gyro, float magneto)
{
	//check each sensor and remember the result (1 = out of range, 0 = OK)
	sensorOutOfRange[0] = checkOneSensor("T", temperature, TEMP_LOW, TEMP_HIGH, "degC");
	sensorOutOfRange[1] = checkOneSensor("P", pressure, pressureLow, pressureHigh, "hPa");
	sensorOutOfRange[2] = checkOneSensor("H", humidity, HUMIDITY_LOW, HUMIDITY_HIGH, "%RH");
	sensorOutOfRange[3] = checkOneSensor("A", accel, ACCEL_LOW, ACCEL_HIGH, "g");
	sensorOutOfRange[4] = checkOneSensor("G", gyro, GYRO_LOW, GYRO_HIGH, "rad/s");
	sensorOutOfRange[5] = checkOneSensor("M", magneto, MAGNETO_LOW, MAGNETO_HIGH, "gauss");
	
	//checks if there is any violations
	int violations = sensorOutOfRange[0] + sensorOutOfRange[1] + sensorOutOfRange[2] + sensorOutOfRange[3] + sensorOutOfRange[4] + sensorOutOfRange[5];
	if (violations == 0) {
		char messageToUART[]="All sensor readings are normal.\r\n";
		sendUART(messageToUART);
	}
	return violations;
}


//packs sensorOutOfRange[] into bits (bit0 T, bit1 P, bit2 H, bit3 A, bit4 G, bit5 M)
uint8_t sensorViolationMask(void) {
	uint8_t mask = 0;
	for (int i = 0; i < 6; i++) {
		if (sensorOutOfRange[i]) mask |= (1 << i);
	}
	return mask;
}

float measurePressure() {
	float groundPressure = 0;
	for (int i = 0; i < 10; i++) {
	    groundPressure = groundPressure + BSP_PSENSOR_ReadPressure();
	    HAL_Delay(50);
	}
	return groundPressure / 10;

}

void readImuMagnetometer(float accelG[3], float magnetoDataFloat[3], float gyroDataRadS[3]){
	int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
	BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
	accelG[0] = (float)accel_data_i16[0]/1000;
	accelG[1] = (float)accel_data_i16[1]/1000;
	accelG[2] = (float)accel_data_i16[2]/1000;

	BSP_GYRO_GetXYZ(gyroDataRadS);   //gyrodata x y z in mdps
	gyroDataRadS[0] = (float)gyroDataRadS[0]*MDPS_TO_RADS;
	gyroDataRadS[1] = (float)gyroDataRadS[1]*MDPS_TO_RADS;
	gyroDataRadS[2] = (float)gyroDataRadS[2]*MDPS_TO_RADS;
	int16_t magnetoData[3];
	BSP_MAGNETO_GetXYZ(magnetoData); //magneto data in milligaus
	magnetoDataFloat[0] = (float)magnetoData[0];
	magnetoDataFloat[1] = (float)magnetoData[1];
	magnetoDataFloat[2] = (float)magnetoData[2];
}

void processMahonyFilter(mahony_t* mahonyFilter, float accelG[3], float magnetoDataFloat[3], float gyroDataRadS[3]){
	static uint32_t gyroBiasInitialiseCounter = 0;
	static uint32_t lastImuTick;
	static float gyroBias[3] = {0};
	if (gyroBiasInitialiseCounter < GYRO_CALIBRATE_COUNT){
		gyroBias[0]+=gyroDataRadS[0];gyroBias[1]+=gyroDataRadS[1];gyroBias[2]+=gyroDataRadS[2];
		gyroBiasInitialiseCounter++;
		if (gyroBiasInitialiseCounter == GYRO_CALIBRATE_COUNT){
			gyroBias[0]/=GYRO_CALIBRATE_COUNT;gyroBias[1]/=GYRO_CALIBRATE_COUNT;gyroBias[2]/=GYRO_CALIBRATE_COUNT;
			lastImuTick = HAL_GetTick();
		}
		return;   //still calibrating: the bias is only a partial sum and dt is not valid yet
	}
	gyroDataRadS[0]-=gyroBias[0];
	gyroDataRadS[1]-=gyroBias[1];
	gyroDataRadS[2]-=gyroBias[2];

	uint32_t currentImuTick = HAL_GetTick();
	float dt = (float)(((float)currentImuTick-(float)lastImuTick)/1000);
	mahony_update(mahonyFilter, gyroDataRadS, accelG, magnetoDataFloat, dt);
	mahony_get_euler(mahonyFilter, &roll,&pitch,&yaw);
	lastImuTick = currentImuTick;

}

void readTPHSensorData(float* tempData, float* pressureData, float* humidityData){
	*tempData = BSP_TSENSOR_ReadTemp();			// read temperature sensor
	*pressureData = BSP_PSENSOR_ReadPressure();  //pressure reading in hectopascal
	*humidityData = BSP_HSENSOR_ReadHumidity();   //humidity data in %rh
}
