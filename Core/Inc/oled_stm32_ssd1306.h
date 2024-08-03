/*
 * oled_stm32_ssd1306.h
 *
 *  Created on: Jul 8, 2024
 *      Author: hp
 */

#ifndef INC_OLED_STM32_SSD1306_H_
#define INC_OLED_STM32_SSD1306_H_

// OLED_STM32_OLED_STM32 library: This library enables interfacing an OLED display based on SSD1306 with an STM32U575VGT6 chip

// Variable Definitions
#define OLED_SPI_COMMAND      0x00
#define OLED_SPI_DATA         0x01
#define OLED_DISPLAY_WIDTH    128
#define OLED_DISPLAY_HEIGHT   64
#define COMMAND_BUFFER_LENGTH 26
#define DISPLAY_BUFFER_LENGTH (OLED_DISPLAY_WIDTH * OLED_DISPLAY_HEIGHT / 8)

// Parameter Definitions
#define OLED_DISPLAYOFF       0xAE
#define OLED_SETCLOCKDIV      0xD5
#define OLED_CLOCKDIVSETTING  0x80
#define OLED_SETMULTIPLEX     0xA8
#define OLED_MULTIPLEXSETTING 0x3F
#define OLED_SETDISPLAYOFFSET 0xD3
#define OLED_DISPLAYOFFSET    0x00
#define OLED_SETSTARTLINE     0x40
#define OLED_CHGPUMPSETTING   0x8D
#define OLED_SETCHGPUMP       0x14
#define OLED_SETADDRESSMODE   0x20
#define OLED_HORZPAGEMODE     0x00
#define OLED_SEGMENTREMAP     0xA1
#define OLED_SCANDIRECTION    0xC8
#define OLED_SETCOMPINS       0xDA
#define OLED_COMPINSSETTING   0x12
#define OLED_SETCONTRAST      0x81
#define OLED_CONTRASTSETTING  0x7F
#define OLED_SETPRECHGPERIOD  0xD9
#define OLED_PRECHGPERIOD     0xF1
#define OLED_SETVCOMHDESELECT 0xDB
#define OLED_VCOMHDESELECTLVL 0x40
#define OLED_DISABLESCROLL    0x2E
#define OLED_FULLDISPLAYOFF   0xA4
#define OLED_SETNORMALDISPLAY 0xA6
#define OLED_DISPLAYON        0xAF

// Function Declarations
void OLED_STM32_initDisplay(void);
void OLED_STM32_sendBuffer(uint8_t *buffer, uint8_t bufferType, uint16_t numberOfElements);
void OLED_STM32_digitalWrite(uint16_t GPIO_Pin, GPIO_PinState PinState);
void OLED_STM32_updateDisplay(void);
void OLED_STM32_clearDisplay(void);
void OLED_STM32_drawPixel(uint8_t x, uint8_t y);
void OLED_STM32_updateMain_DISCONNECTEDView(void);
void OLED_STM32_updateMain_CHARGINGView(void);
void OLED_STM32_updateMain_FAULTView(void);
void OLED_STM32_updateMain_BienvenueView(void);
void OLED_STM32_updateMainView(void);
void OLED_STM32_drawMonospaceCharacter(uint8_t xPosOffset, uint8_t yPosOffset, uint8_t myChar);
void OLED_STM32_drawMonospaceString(uint8_t xPos, uint8_t yPos, const char* myString);
uint8_t OLED_STM32_getMonospaceGlyphIndex(uint8_t charIndex);
void OLED_STM32_drawLine(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd);
void OLED_STM32_drawLargeCharacter(uint8_t xPosOffset, uint8_t yPosOffset, uint8_t myChar);
void OLED_STM32_drawLargeString(uint8_t xPos, uint8_t yPos, const char* myString);
uint8_t OLED_STM32_getLargeGlyphIndex(uint8_t charIndex);
void OLED_STM32_drawImage(uint8_t xPosOffset, uint8_t yPosOffset);

#endif /* INC_OLED_STM32_SSD1306_H_ */

