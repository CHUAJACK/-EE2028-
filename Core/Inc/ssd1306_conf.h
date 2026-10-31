#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// Our board uses an STM32L4 chip
#define STM32L4

// The Grove OLED talks over I2C
#define SSD1306_USE_I2C

// Which I2C handle and address to use (must match I2C1_Init in main.c)
#define SSD1306_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        (0x3C << 1)

// Fonts to include (each one uses flash memory)
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18

#endif
