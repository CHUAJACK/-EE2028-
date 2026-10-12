  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "stdio.h"
#include "math.h"



#define MG_TO_GMS2 9.8/1000.0f
#define BUTTON_DOUBLE_PRESS_TIME 500

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
static void MX_GPIO_Init(void);

//-----------Determine Sensor Validity----------//
//setting the thresholds
//temperature in degC: safe operating range for Li-ion battery and electronics
#define TEMP_LOW       0.0f
#define TEMP_HIGH      45.0f
//humidity in %RH: comfortable range for humans
#define HUMIDITY_LOW   30.0f
#define HUMIDITY_HIGH  70.0f
//acceleration in m/s^2: below 0.5*9.8 = free fall, above 2*9.8 = collision
#define ACCEL_LOW      4.9f
#define ACCEL_HIGH     19.6f
//rotation in dps: above 250 dps = drone is tumbling
#define GYRO_LOW       0.0f
#define GYRO_HIGH      250.0f
//magnetic field in gauss: Earth's field is about 0.25 to 0.65 gauss
#define MAGNETO_LOW    0.20f
#define MAGNETO_HIGH   0.80f


//for pressure, the pressure difference relative to takeoff is calculated (range is -10m to 120m) because pressure differs too much 
//pressure drops about 0.12 hPa for every metre the drone climbs, so
#define PRESSURE_DROP_FOR_MAX_HEIGHT  14.4f
#define PRESSURE_RISE_FOR_MIN_HEIGHT  1.2f

float pressureLow;
float pressureHigh;

//checks one sensor reading against its low and high limit
//prints a message and returns 1 if the reading is out of range, otherwise returns 0
int checkOneSensor(char *name, float reading, float low, float high, char *unit)
{
	if (reading > high) {
		printf("%s: %.2f (%s), exceeds threshold of %.2f (%s).\r\n", name, reading, unit, high, unit);
		return 1;
	}
	if (reading < low) {
		printf("%s: %.2f (%s), is below threshold of %.2f (%s).\r\n", name, reading, unit, low, unit);
		return 1;
	}
	return 0;
}

//checks all six sensors
int determineSensorValidity(float temperature, float pressure, float humidity, float accel, float gyro, float magneto)
{
	int violations = 0;
	violations = violations + checkOneSensor("T", temperature, TEMP_LOW, TEMP_HIGH, "degC");
	violations = violations + checkOneSensor("P", pressure, pressureLow, pressureHigh, "hPa");
	violations = violations + checkOneSensor("H", humidity, HUMIDITY_LOW, HUMIDITY_HIGH, "%RH");
	violations = violations + checkOneSensor("A", accel, ACCEL_LOW, ACCEL_HIGH, "m/s^2");
	violations = violations + checkOneSensor("G", gyro, GYRO_LOW, GYRO_HIGH, "dps");
	violations = violations + checkOneSensor("M", magneto, MAGNETO_LOW, MAGNETO_HIGH, "gauss");
	
	//checks if there is any violations
	if (violations == 0) {
		printf("All sensor readings are normal.\r\n");
	}
	return violations;
}


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

	//measure the average ground pressure and find lower and upper bound
	float groundPressure = 0;
	for (int i = 0; i < 10; i++) {
    	groundPressure = groundPressure + BSP_PSENSOR_ReadPressure();
    	HAL_Delay(50);
	}
	groundPressure = groundPressure / 10;
	pressureLow  = groundPressure - PRESSURE_DROP_FOR_MAX_HEIGHT;
	pressureHigh = groundPressure + PRESSURE_RISE_FOR_MIN_HEIGHT;



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

				//converts x y z values to a single vector
				float accelTotal = sqrtf(accel_data[0]*accel_data[0] + accel_data[1]*accel_data[1] + accel_data[2]*accel_data[2]);
				float gyroTotal = sqrtf(gyroData[0]*gyroData[0] + gyroData[1]*gyroData[1] + gyroData[2]*gyroData[2]) / 1000;
				float magnetoTotal = sqrtf((float)magnetoData[0]*magnetoData[0] + (float)magnetoData[1]*magnetoData[1] + (float)magnetoData[2]*magnetoData[2]) / 1000;
				//checks the sensor validity
				determineSensorValidity(tempData, pressureData, humidityData, accelTotal, gyroTotal, magnetoTotal);
			
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
