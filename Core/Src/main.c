  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "stdio.h"
#define MG_TO_GMS2 9.8/1000.0f


extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)




typedef enum {
	standby = 0 ,
	battle 	= 1,
	battleTheLastOf2028 = 2
} droneModes;


int main(void)
{
	droneModes droneMode;
	droneMode = standby;
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	uint32_t lastSensorTimeTick = HAL_GetTick();
	uint32_t tickStart;
	while (1)
	{
		tickStart = HAL_GetTick();  //milliseconds

		//read sensor messages
		if (droneMode == standby) {
		if ((tickStart-lastSensorTimeTick)>=1000){
				float tempData;
				tempData = BSP_TSENSOR_ReadTemp();			// read temperature sensor

				float accel_data[3];
				int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
				BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
				accel_data[0] = (float)accel_data_i16[0] * MG_TO_GMS2;
				accel_data[1] = (float)accel_data_i16[1] * MG_TO_GMS2;
				accel_data[2] = (float)accel_data_i16[2] * MG_TO_GMS2;

				float pressureData ;
				pressureData = BSP_PSENSOR_ReadPressure();  //pressure reading in hectopascal

				float gyroData[3];
				BSP_GYRO_GetXYZ(gyroData);   //gyrodata x y z in mdps

				float humidityData;
				humidityData = BSP_HSENSOR_ReadHumidity();   //humidity data in %rh

				int16_t magnetoData[3];
				BSP_MAGNETO_GetXYZ(magnetoData); //magneto data in milligaus

				lastSensorTimeTick = HAL_GetTick();  //update last tick
				printf("T: %f (C), P: %f (hPa), H: %f (rh)\nAx: %f; Ay: %f; Az: %f\nAx: %f; Ay: %f; Az: %f\nMx: %d; My: %d; Mz: %d\n", tempData, pressureData, humidityData, accel_data[0], accel_data[1], accel_data[2], gyroData[0], gyroData[1], gyroData[2], magnetoData[0], magnetoData[1], magnetoData[2]);
			}
		} else{
			if (droneMode == battle){
				float notimplementedyet;
			}
		}

	}

}
