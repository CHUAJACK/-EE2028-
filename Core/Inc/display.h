/**
 * @file    display.h
 * @brief   OLED pages for the drone (standby sensor status, battle laser charge).
 */
#ifndef DISPLAY_H
#define DISPLAY_H

void drawSensorStatusPage(void);
void drawLaserPage(void);
void updateDisplay(void);   //clears the screen, draws the page for the current mode and sends it to the OLED

#endif /* DISPLAY_H */
