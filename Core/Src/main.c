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
#include "string.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "ble_drone.h"
#include "sensors.h"
#include "display.h"

#define BLE_STATUS_PERIOD_MS 250   //how often mode + orientation are sent over bluetooth

#define MG_TO_GMS2 (9.8f/1000.0f)
#define BUTTON_DOUBLE_PRESS_TIME 500
#define SWITCH_I2C_ADDR   (0x03 << 1)   //5-way switch I2C address
#define KEY_LEFT   1
#define KEY_RIGHT  3
#define M_KP	8.0
#define M_KI 	0.05
#define IMU_PERIOD_MS 20

#include "mahony.h"  //include is placed here to use the define macro for gravity

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
static void MX_GPIO_Init(void);
void buzzerToggleTask(uint8_t toggleBuzzer);

//-----------LASERRRRRRRRRR----------//
int laserCharge = 0;
uint32_t lastOf2028StartTick = 0;   //tick when last of 2028 mode started (drives the countdown)
droneModes droneMode;

void chargeLaser() {
	laserCharge += LASER_CHARGE_PER_PRESS;
	if (laserCharge > LASER_MAX) {
		laserCharge = LASER_MAX;
	}
	printf("Laser Charging! energy: %d/10\r\n", laserCharge);
	char messageToUART[60];
	sprintf(messageToUART, "Laser Charging! energy: %d/10\r\n", laserCharge);
	sendUART(messageToUART);
}

//returns 1 if the laser fired, 0 if there was not enough charge
int shootLaser() {
	if (laserCharge < LASER_COST) {
		printf("Insufficient power! energy: %d/10\r\n", laserCharge);
		char messageToUART[60];
		sprintf(messageToUART, "Insufficient power! energy: %d/10\r\n", laserCharge);
		sendUART(messageToUART);
		return 0;
	} else {
		laserCharge -= LASER_COST;
	}
	printf("Laser Fired! energy: %d/10\r\n", laserCharge);
	char messageToUART[60];
	sprintf(messageToUART, "Laser Fired! energy: %d/10\r\n", laserCharge);
	sendUART(messageToUART);
	buzzerToggleTask(1);
	return 1;
}

//-----------LASERRRRRRRRRR----------//

UART_HandleTypeDef huart1;

void sendUART(char* message){
	uint16_t messageSize=strlen(message);
	HAL_UART_Transmit(&huart1, (uint8_t*)message, messageSize,0xFFFF);
}

static void UART1_Init(void)
{
	/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while(1);
	}

}
I2C_HandleTypeDef hi2c1;

//sets up I2C1 on PB8 (SCL) and PB9 (SDA)
static void I2C1_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;        //I2C pins must be open-drain
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;      //connect the pins to I2C1
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00000E14;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	HAL_I2C_Init(&hi2c1);
}



//void fiveWaySwitchInit(void) {
//	uint8_t command = 0x02;
//	HAL_I2C_Master_Transmit(&hi2c1, SWITCH_I2C_ADDR, &command, 1, 100);
//}

//returns which key (0 to 4) was clicked, or -1 if nothing was clicked
//int readFiveWaySwitch(void) {
//	uint8_t command = 0x01;   //"give me your events"
//	uint8_t data[9];
//
//	if (HAL_I2C_Master_Transmit(&hi2c1, SWITCH_I2C_ADDR, &command, 1, 100) != HAL_OK) {
//		return -1;
//	}
//	if (HAL_I2C_Master_Receive(&hi2c1, SWITCH_I2C_ADDR, data, 9, 100) != HAL_OK) {
//		return -1;
//	}
//
//	if ((data[3] & 0x80) == 0) {
//		return -1;   //no event happened
//	}
//
//	for (int i = 0; i < 5; i++) {
//		if (data[4 + i] & 0x02) {
//			return i;   //bit 1 = single click on key i
//		}
//	}
//	return -1;
//}




static void MX_GPIO_Init(void){
	__HAL_RCC_GPIOC_CLK_ENABLE();
	//BUTTON_EXTI13 PC13
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	__HAL_RCC_GPIOA_CLK_ENABLE();
	//BUZZER GPIO OUTPUT PA4
	GPIO_InitTypeDef GPIO_InitStruct_BUZZER = {0};
	GPIO_InitStruct_BUZZER.Pin = ARD_D7_Pin;
	GPIO_InitStruct_BUZZER.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct_BUZZER.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct_BUZZER);

}

volatile uint32_t buttonPressCount = 0;   //changed in the button interrupt
volatile uint32_t buttonLastPress;

// callback for all EXTI interrupts: the button and the bluetooth module's IRQ line
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == SPBTLE_RF_IRQ_EXTI6_Pin) {
		BLE_IrqHandler();   //bluetooth module has data for us
	} else if (GPIO_Pin == BUTTON_EXTI13_Pin) {
		buttonPressCount +=1;
		buttonLastPress = HAL_GetTick();
	}
}

uint32_t processButton(uint32_t currentTick){
	if (buttonPressCount == 0) return 0;
	//if its pressed only once, wait for the timeout before processing so that the second press comes in
	if (buttonPressCount == 1 && (currentTick-buttonLastPress) > BUTTON_DOUBLE_PRESS_TIME){
		buttonPressCount = 0;
		return 1;
	}
	//if its pressed twice (or more, e.g. while the loop was busy), process it immediately
	if (buttonPressCount >= 2){
		buttonPressCount = 0;
		return 2;
	}
	return 0;
}
// on -> off 1 period
void processLEDBlinkRate(droneModes droneMode, uint32_t currentTick){
	static uint32_t lastToggleTime = 0;   //tick of the last LED change
	if (droneMode == 0){
		BSP_LED_On(LED2);
		lastToggleTime = currentTick;
	} else if ((currentTick-lastToggleTime)>(1000/((int)droneMode*2))){
		BSP_LED_Toggle(LED2);
		lastToggleTime = currentTick;
	}
}


void buzzerToggleTask(uint8_t toggleBuzzer){
	static uint8_t buzzerState = 0;
	static uint32_t lastBuzzerToggleTime = 0;
	uint32_t currentTick = HAL_GetTick();

	if (buzzerState == 0 && toggleBuzzer == 1){
		HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 1);  //switch to high
		lastBuzzerToggleTime = currentTick;
		buzzerState = 1;
	} else if (buzzerState == 1 && currentTick-lastBuzzerToggleTime > 500){   //only end a beep this task started
		HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 0);  //switch to low
		lastBuzzerToggleTime = currentTick;
		buzzerState = 0;
	}
}

void doubleBeepTask(uint8_t toggleBuzzer){
	static uint8_t buzzerState = 0; //1 is short high 2 is short low 3 is short high 0
	static uint32_t lastBuzzerToggleTime = 0;
	static uint32_t firstBuzzerToggleTime = 0;
	uint32_t currentTick = HAL_GetTick();
	static uint8_t active = 0;
	if (buzzerState == 0 && toggleBuzzer == 1){
		HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 1);  //switch to high
		lastBuzzerToggleTime = currentTick;
		firstBuzzerToggleTime = lastBuzzerToggleTime;
		buzzerState = 1;
		active = 1;
	}
	if (toggleBuzzer == 3){
		active = 0;
		HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 0);  //switch to low
		buzzerState = 0;
	}
	if (active == 1){
		switch (buzzerState){
		case 1:
			if (currentTick-lastBuzzerToggleTime >= 150){
				HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 0);  //switch to low
				lastBuzzerToggleTime = currentTick;
				buzzerState = 2;
			}
			break;
		case 2:
			if (currentTick-lastBuzzerToggleTime >= 50){
				HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 1);  //switch to high
				lastBuzzerToggleTime = currentTick;
				buzzerState = 3;
			}
			break;
		case 3:
			if (currentTick-lastBuzzerToggleTime >= 150){
				HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 0);  //switch to low
				lastBuzzerToggleTime = currentTick;
				buzzerState = 4;
			}
			break;
		case 4:
			if (currentTick-firstBuzzerToggleTime >= 1000){
				HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 1);  //switch to high
				lastBuzzerToggleTime = currentTick;
				buzzerState = 1;
				firstBuzzerToggleTime = currentTick;
			}
			break;
		}
	}

}


int main(void)
{

	droneMode = standby;
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	MX_GPIO_Init();
	UART1_Init();
	sendUART("STANDBY MODE\r\n");

	/* Peripheral initializations using BSP functions */

	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_LED_Init(LED2);

	I2C1_Init();
	ssd1306_Init();
	updateDisplay();

	//find lower and upper bound for pressure
	float groundPressure = measurePressure();
	pressureLow  = groundPressure - PRESSURE_DROP_FOR_MAX_HEIGHT;
	pressureHigh = groundPressure + PRESSURE_RISE_FOR_MIN_HEIGHT;

	//start bluetooth; if it fails the drone still runs, just without telemetry over BLE
	char bleMessage[80];
	int bleError = BLE_Init();
	if (bleError == 0) {
		char bleAddress[18];
		BLE_GetAddress(bleAddress);
		sprintf(bleMessage, "Bluetooth ready, advertising as %s (%s)\r\n", BLE_DEVICE_NAME, bleAddress);
	} else {
		sprintf(bleMessage, "Bluetooth init failed at step %d, continuing without BLE\r\n", bleError);
	}
	printf("%s", bleMessage);
	sendUART(bleMessage);
	BLE_UpdateLaser(laserCharge, LASER_MAX, LASER_COST, LASER_CHARGE_PER_PRESS, BLE_LASER_EVT_NONE);

	mahony_t mahonyFilter;

	mahony_init(&mahonyFilter,M_KP, M_KI);

	float accelG[3];
	float magnetoDataFloat[3];
	float gyroDataRadS[3];
	float tempData;
	float pressureData;
	float humidityData;

	uint32_t lastImuTick=HAL_GetTick();
	int lastCountdownShown = -1;   //last "self destructing in ..." number sent

	uint32_t lastSensorTimeTick = HAL_GetTick();
	uint32_t lastBleStatusTick = HAL_GetTick();
	uint32_t tickStart;
	uint32_t lastDisplayTick = HAL_GetTick();
	uint32_t buttonPressResult;

	while (1)
	{
		tickStart = HAL_GetTick();  //milliseconds
		//flashes every 1s
		if (tickStart - lastDisplayTick >= 1000) {
			updateDisplay();
			lastDisplayTick = tickStart;
		}
		buzzerToggleTask(0);
		doubleBeepTask(0);
		//process button press and LED flash rate
		processLEDBlinkRate(droneMode, tickStart);
		buttonPressResult = processButton(tickStart);

		//IMU reading for mahony filter
		if ((tickStart - lastImuTick)>IMU_PERIOD_MS){
			readImuMagnetometer(accelG, magnetoDataFloat, gyroDataRadS);
			processMahonyFilter(&mahonyFilter, accelG, magnetoDataFloat, gyroDataRadS);
			lastImuTick = tickStart;
		}

		//bluetooth: handle connect/disconnect, and send mode + orientation in every mode
		BLE_Process();
		if ((tickStart - lastBleStatusTick) >= BLE_STATUS_PERIOD_MS) {
			BLE_UpdateStatus((uint8_t)droneMode, roll, pitch, yaw);
			lastBleStatusTick = tickStart;
		}

		switch (droneMode) {
		case standby: {
			if (buttonPressResult == 2) {
				//transition to battle mode
				printf("BATTLE MODE\r\n");
				sendUART("BATTLE MODE\r\n");
				droneMode=battle;continue;
			}

			if ((tickStart-lastSensorTimeTick)>=1000){
				readTPHSensorData(&tempData, &pressureData, &humidityData);
				//converts x y z values to a single vector
				float accelTotal = sqrtf(accelG[0]*accelG[0] + accelG[1]*accelG[1] + accelG[2]*accelG[2]);
				float gyroTotal = sqrtf(gyroDataRadS[0]*gyroDataRadS[0] + gyroDataRadS[1]*gyroDataRadS[1] + gyroDataRadS[2]*gyroDataRadS[2]);
				float magnetoTotal = sqrtf((float)magnetoDataFloat[0]*magnetoDataFloat[0] + (float)magnetoDataFloat[1]*magnetoDataFloat[1] + (float)magnetoDataFloat[2]*magnetoDataFloat[2]) / 1000;
				//checks the sensor validity
				determineSensorValidity(tempData, pressureData, humidityData, accelTotal, gyroTotal, magnetoTotal);
				//publish the same readings over bluetooth (standby only)
				BLE_UpdateSensors(tempData, pressureData, humidityData, accelTotal, gyroTotal, magnetoTotal, sensorViolationMask());
				lastSensorTimeTick = HAL_GetTick();  //update last tick
			}
			break;
		}

		case battle: {
			if(buttonPressResult == 1) {
				chargeLaser();
				int fired = shootLaser();
				updateDisplay();
				BLE_UpdateLaser(laserCharge, LASER_MAX, LASER_COST, LASER_CHARGE_PER_PRESS, fired ? BLE_LASER_EVT_FIRED : BLE_LASER_EVT_NO_POWER);
			}
			if (buttonPressResult == 2) {
				printf("STANDBY MODE\r\n");
				sendUART("STANDBY MODE\r\n");
				droneMode=standby;continue;
			}
			if (roll > 120 || roll < -120) {
				droneMode=battleTheLastOf2028;
				lastOf2028StartTick=tickStart;
				lastCountdownShown = -1;
				sendUART("Drone is under attack! entering last of 2028 mode\r\n");
				doubleBeepTask(1);
				continue;
			}
			break;
		}

		case battleTheLastOf2028: {
			if (tickStart-lastOf2028StartTick > LAST_OF_2028_TIME_MS){
				droneMode=destroyed;
				BLE_UpdateStatus((uint8_t)droneMode, roll, pitch, yaw);   //last update: tell the laptop the drone is destroyed
				BLE_Shutdown();   //then switch bluetooth off for good
				sendUART("Bluetooth switched off\r\n");
				sendUART("Kaboom!\r\n");
				doubleBeepTask(3);
				break;   //leaves the switch; the check below leaves the loop
			}
			//count down 10, 9, ... 1: one message each time the whole seconds left change
			int secondsLeft = (int)((LAST_OF_2028_TIME_MS - (tickStart - lastOf2028StartTick)) / 1000);
			if (secondsLeft > 0 && secondsLeft != lastCountdownShown){
				lastCountdownShown = secondsLeft;
				updateDisplay();   //show the new countdown number on the OLED
				char messageToUART[60];
				sprintf(messageToUART, "Drone is self destructing in ...%d\r\n", secondsLeft);
				sendUART(messageToUART);
			}

			if (buttonPressResult == 2) {
				droneMode=battle;
				sendUART("Drone is Rescued.. returning to battle mode\r\n");
				doubleBeepTask(3);
				continue;
			}
			break;
		}

		case destroyed:
		default:
			break;
		}

		if (droneMode == destroyed) break;   //a break inside the switch only leaves the switch
	}
	   ssd1306_SetDisplayOn(0);   //terminate the display
	while (1){
	}

}
