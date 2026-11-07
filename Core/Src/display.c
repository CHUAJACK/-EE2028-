/**
 * @file    display.c
 * @brief   OLED pages for the drone (standby sensor status, battle laser charge).
 */
#include "display.h"
#include "main.h"
#include "sensors.h"
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"

void drawSensorStatusPage(void) {
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("STANDBY", Font_6x8, White);
	for (int i = 0; i < 6; i++) {
		int column = i % 3;          //0, 1, 2, 0, 1, 2
		int row = i / 3;             //0, 0, 0, 1, 1, 1
		int x = column * 43;
		int y = 14 + row * 25;
		if (sensorOutOfRange[i] == 1) {
			//out of threshold: solid box, black text
			ssd1306_FillRectangle(x, y, x + 40, y + 22, White);
			ssd1306_SetCursor(x + 6, y + 7);
			ssd1306_WriteString(sensorLabels[i], Font_7x10, Black);
		} else {
			//within threshold: outline only, white text
			ssd1306_DrawRectangle(x, y, x + 40, y + 22, White);
			ssd1306_SetCursor(x + 6, y + 7);
			ssd1306_WriteString(sensorLabels[i], Font_7x10, White);
		}
	}
}
void drawLaserPage(void) {
	char line[22];
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("BATTLE - LASER", Font_6x8, White);
	sprintf(line, "Energy: %d/10", laserCharge);
	ssd1306_SetCursor(0, 20);
	ssd1306_WriteString(line, Font_7x10, White);
	ssd1306_DrawRectangle(0, 40, 101, 50, White);
	if (laserCharge > 0) {
		ssd1306_FillRectangle(1, 41, laserCharge * 10, 49, White);
	}
}

void updateDisplay(void) {
	ssd1306_Fill(Black);
	if (droneMode == standby) {
		drawSensorStatusPage();
	} else {
		drawLaserPage();    //battle and Last of EE2028
	}
	ssd1306_UpdateScreen();
}
