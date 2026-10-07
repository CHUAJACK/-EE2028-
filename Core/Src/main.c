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
#define acceleration_count_target 2// accel count = period/500ms
#define temperature_count_target 3 // temperature count target = period/500ms

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

int main(void)
{
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	uint16_t accel_count = 0;
	uint16_t temp_count = 0;
	while (1)
	{

		accel_count+=1;
		temp_count+=1;
		if (accel_count== acceleration_count_target ){
			accel_count=0;
			float accel_data[3];
			int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
			// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
			// Converting to float to print the actual acceleration.
			accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
			accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
			accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);
			printf("Accel X : %f; Accel Y : %f; Accel Z : %f\n", accel_data[0], accel_data[1], accel_data[2]);
		}
		if (temp_count== temperature_count_target ){
			temp_count=0;
			float temp_data;
			temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor
			printf("Temperature : %f\n", temp_data);

		}


		HAL_Delay(500);	// read once a ~second.

	}

}
