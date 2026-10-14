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
#include <stdbool.h>
#include "math.h"



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

//-----------LASERRRRRRRRRR----------//
#define LASER_CHARGE_PER_PRESS 3
#define LASER_MAX 10
#define LASER_COST 5

int laserCharge = 0;

void chargeLaser() {
	laserCharge += LASER_CHARGE_PER_PRESS;
	if (laserCharge > LASER_MAX) {
		laserCharge = LASER_MAX;
	}
	printf("Laser Charging! energy: %d/10\r\n", laserCharge);
}

void shootLaser() {
	if (laserCharge < LASER_COST) {
		printf("Insufficient power! energy: %d/10\r\n", laserCharge);
		return;
	} else {
		laserCharge -= LASER_COST;
	}
	printf("Laser Fired! energy: %d/10\r\n", laserCharge);
}

//-----------LASERRRRRRRRRR----------//

//-----------Determine Sensor Validity----------//
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
#define GYRO_LOW       0.0f * MDPS_TO_RADS
#define GYRO_HIGH       250000.0f * MDPS_TO_RADS
//magnetic field in gauss: Earth's field is about 0.25 to 0.65 gauss
#define MAGNETO_LOW    0.20f
#define MAGNETO_HIGH   2.50f


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

//-----------Determine Sensor Validity----------//


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

float measurePressure() {
	float groundPressure = 0;
	for (int i = 0; i < 10; i++) {
	    groundPressure = groundPressure + BSP_PSENSOR_ReadPressure();
	    HAL_Delay(50);
	}
	return groundPressure / 10;

}

uint32_t processButton(uint32_t currentTick){
	if (buttonPressCount == 0) return 0;
	//if its pressed only once, wait for the timeout before processing so that the second press comes in
	if (buttonPressCount == 1 && (currentTick-buttonLastPress) > BUTTON_DOUBLE_PRESS_TIME){
		buttonPressCount = 0;
		return 1;
	}
	//if its pressed twice, process it immediately
	if (buttonPressCount == 2){
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

	//find lower and upper bound for pressure
	float groundPressure = measurePressure();
	pressureLow  = groundPressure - PRESSURE_DROP_FOR_MAX_HEIGHT;
	pressureHigh = groundPressure + PRESSURE_RISE_FOR_MIN_HEIGHT;	

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
		if (buttonPressResult == 2) {
			//transition to battle mode
			printf("BATTLE MODE\r\n");
			droneMode=battle;continue;
		}

		if ((tickStart-lastSensorTimeTick)>=500){
				readTPHSensorData(&tempData, &pressureData, &humidityData);
				//converts x y z values to a single vector
				float accelTotal = sqrtf(accelG[0]*accelG[0] + accelG[1]*accelG[1] + accelG[2]*accelG[2]);
				float gyroTotal = sqrtf(gyroDataRadS[0]*gyroDataRadS[0] + gyroDataRadS[1]*gyroDataRadS[1] + gyroDataRadS[2]*gyroDataRadS[2]);
				float magnetoTotal = sqrtf((float)magnetoDataFloat[0]*magnetoDataFloat[0] + (float)magnetoDataFloat[1]*magnetoDataFloat[1] + (float)magnetoDataFloat[2]*magnetoDataFloat[2]) / 1000;
				//checks the sensor validity
				determineSensorValidity(tempData, pressureData, humidityData, accelTotal, gyroTotal, magnetoTotal);
			
				lastSensorTimeTick = HAL_GetTick();  //update last tick
				//printf("T: %f (C), P: %f (hPa), H: %f (rh)\nAx: %d; Ay: %d; Az: %d (g)\nGx: %d; Gy: %d; Gz: %d (rads-1)\nMx: %d; My: %d; Mz: %d  (mG)\n",
				//tempData, pressureData, humidityData, accel_data[0], accel_data[1], accel_data[2], gyroData[0], gyroData[1], gyroData[2], magnetoData[0], magnetoData[1], magnetoData[2]);
		}
		} else{
			if (droneMode == battle){
				if(buttonPressResult == 1) {
					chargeLaser();
					shootLaser();
				}
				if (buttonPressResult == 2) {
					printf("STANDBY MODE\r\n");
					droneMode=standby;continue;
				}
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
