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
#include <stdbool.h>


#define MG_TO_GMS2 9.8/1000.0f
#define BUTTON_DOUBLE_PRESS_TIME 500

#define M_KP	8.0
#define M_KI 	0.05
#define MDPS_TO_RADS 2*3.1415926535/180000.0f
#define GYRO_CALIBRATE_COUNT 20
#define IMU_PERIOD_MS 20

#include "mahony.h"  //include is placed here to use the define macro for gravity

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
static void MX_GPIO_Init(void);

float roll;
float pitch;
float yaw;

typedef enum {
	standby = 0 ,
	battle 	= 1,
	battleTheLastOf2028 = 2,   //implicitly used as led blink rate
	destroyed = 100
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
droneModes droneMode;

// callback for button
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
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

int main(void)
{

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

	mahony_t mahonyFilter;

	mahony_init(&mahonyFilter,M_KP, M_KI);

	float accelG[3];
	float magnetoDataFloat[3];
	float gyroDataRadS[3];
	float tempData;
	float pressureData;
	float humidityData;

	uint32_t lastImuTick=HAL_GetTick();
	uint32_t lastOf2028StartTick = 0;

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

		//IMU reading for mahony filter
		if ((tickStart - lastImuTick)>IMU_PERIOD_MS){
			readImuMagnetometer(accelG, magnetoDataFloat, gyroDataRadS);
			processMahonyFilter(&mahonyFilter, accelG, magnetoDataFloat, gyroDataRadS);
			lastImuTick = tickStart;
		}

		if (droneMode == standby) {
		if (buttonPressResult == 2) {droneMode=battle;continue;} //transition to battle mode

		if ((tickStart-lastSensorTimeTick)>=500){
				readTPHSensorData(&tempData, &pressureData, &humidityData);
				lastSensorTimeTick = HAL_GetTick();  //update last tick
				//printf("T: %f (C), P: %f (hPa), H: %f (rh)\nAx: %d; Ay: %d; Az: %d (g)\nGx: %d; Gy: %d; Gz: %d (rads-1)\nMx: %d; My: %d; Mz: %d  (mG)\n",
				//tempData, pressureData, humidityData, accel_data[0], accel_data[1], accel_data[2], gyroData[0], gyroData[1], gyroData[2], magnetoData[0], magnetoData[1], magnetoData[2]);
		}
		} else{
			if (droneMode == battle){
				if (buttonPressResult == 2) {droneMode=standby;continue;}
				if (roll > 120 || roll < -120) {droneMode=battleTheLastOf2028;lastOf2028StartTick=tickStart;continue;}
			} else {
				//last of 2028 drone mode
				if (tickStart-lastOf2028StartTick > 10000){droneMode=destroyed;break;}
				if (buttonPressResult == 2) {droneMode=battle;continue;}

			}
		}

	}
	while (1){

	}

}
