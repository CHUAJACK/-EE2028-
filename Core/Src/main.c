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
#define BUTTON_DOUBLE_PRESS_TIME 500

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
static void MX_GPIO_Init(void);



typedef enum {
	standby = 0 ,
	battle 	= 1,
	battleTheLastOf2028 = 2
} droneModes;

//LED2 IS PB14


static void MX_GPIO_Init(void){
	__HAL_RCC_GPIOC_CLK_ENABLE();

	//BUTTON_EXTI13 PC13
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);




	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint32_t buttonPressCount;
uint32_t buttonPressCount = 0;
uint32_t buttonLastPress;


// callback for button
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	printf('buttonPressed');
	buttonPressCount +=1;
	buttonLastPress = HAL_GetTick();

}

uint32_t processButton(uint32_t currentTick){
	if (buttonPressCount == 0) return 0;
	//if its pressed only once, wait for the timeout before processing so that the second press comes in
	if (buttonPressCount == 1 && (currentTick-buttonLastPress) > BUTTON_DOUBLE_PRESS_TIME){
		printf("button 1");
		buttonPressCount = 0;
		return 1;
	}
	//if its pressed twice, process it immediately
	if (buttonPressCount == 2){
		printf("button 2");
		buttonPressCount = 0;
		return 2;
	}
	return 0;
}
// on -> off 1 period
uint32_t processLEDBlinkRate(droneModes droneMode, uint32_t currentTick, uint32_t lastToggleTime){
	if (droneMode == 0){
		BSP_LED_On(LED2);
		return currentTick;
	} else if ((currentTick-lastToggleTime)>(1000/((int)droneMode*2))){
		BSP_LED_Toggle(LED2);
		return currentTick;
	}
	return lastToggleTime;
}


int main(void)
{
	droneModes droneMode;
	droneMode = standby;
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	MX_GPIO_Init();

	/* Peripheral initializations using BSP functions */

	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_LED_Init(LED2);





	uint32_t lastSensorTimeTick = HAL_GetTick();
	uint32_t ledLastToggleTime = HAL_GetTick();
	uint32_t tickStart;
	uint32_t buttonPressResult;
	while (1)
	{

		tickStart = HAL_GetTick();  //milliseconds

		//process button press and LED flash rate
		ledLastToggleTime = processLEDBlinkRate(droneMode, tickStart, ledLastToggleTime);
		buttonPressResult = processButton(tickStart);

		//read sensor messages

		if (droneMode == standby) {
		if (buttonPressResult == 2) {droneMode=battle;continue;} //transition to battle mode
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
				//printf("T: %f (C), P: %f (hPa), H: %f (rh)\nAx: %f; Ay: %f; Az: %f\nAx: %f; Ay: %f; Az: %f\nMx: %d; My: %d; Mz: %d\n", tempData, pressureData, humidityData, accel_data[0], accel_data[1], accel_data[2], gyroData[0], gyroData[1], gyroData[2], magnetoData[0], magnetoData[1], magnetoData[2]);
			}
		} else{
			if (droneMode == battle){
				if (buttonPressResult == 2) {droneMode=standby;continue;}
			} else {
				//last of 2028 drone mode
			}
		}

	}

}
