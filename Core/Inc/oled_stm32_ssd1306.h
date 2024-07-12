/*
 * oled_stm32_ssd1306.h
 *
 *  Created on: Jul 8, 2024
 *      Author: hp
 */

#ifndef INC_OLED_STM32_SSD1306_H_
#define INC_OLED_STM32_SSD1306_H_



#endif /* INC_OLED_STM32_SSD1306_H_ */
// OLED_STM32_OLED_STM32 library: This library shall enable interfacing an OLED_STM32 OLED based display on an STM3U575vgt6 chip

// Variable Definitions
#define    OLED_SPI_COMMAND      0x00
#define    OLED_SPI_DATA         0x01
#define    OLED_DISPLAY_WIDTH    128
#define    OLED_DISPLAY_HEIGHT   64
#define    COMMAND_BUFFER_LENGTH 26
#define    DISPLAY_BUFFER_LENGTH OLED_DISPLAY_WIDTH * OLED_DISPLAY_HEIGHT / 8


// Parameter Definitions in correct order
#define    OLED_DISPLAYOFF       0xAE // turn display off
#define    OLED_SETCLOCKDIV      0xD5 // clock divide ratio
#define    OLED_CLOCKDIVSETTING  0x80 // oscillator frequency DCLK=9Mhz;Fosc=1000b->9Mhz;D=1
#define    OLED_SETMULTIPLEX     0xA8 // set MUX ratio(defines how many COM lines are used)
#define    OLED_MULTIPLEXSETTING 0x3F // MUX ratio depends on height [0x1F = 32px] or [0x3F = 64px]
#define    OLED_SETDISPLAYOFFSET 0xD3 // Display Offset(specifies the mapping of the display start line to one of COM0~COM63)
#define    OLED_DISPLAYOFFSET    0x00 // Display Offset = 0 RAM
#define    OLED_SETSTARTLINE     0x40 // Set Start Line 0 (display line)
#define    OLED_CHGPUMPSETTING   0x8D // Set Charge Pump Setting
#define    OLED_SETCHGPUMP       0x10 // Charge Pump Setting [0x10 = Disable] or [0x14 = Enable=32px]
#define    OLED_SETADDRESSMODE   0x20 // Page Addressing Mode
#define    OLED_HORZPAGEMODE     0x00 // Horizontal with auto new line
#define    OLED_SEGMENTREMAP     0xA1 // Segment Remap A0(Normal display, left to right) / A1 (right to left ,Mirroring Horizontally)
#define    OLED_SCANDIRECTION    0xC8 // Scan Direction C0 (Normal sweep, up and down.)/ C8 (Reverse sweep, bottom to top,Mirroring Vertically)
#define    OLED_SETCOMPINS       0xDA // Set COM pin HW config
#define    OLED_COMPINSSETTING   0x12 // sequential com pin config [0x02 = 32px] or [0x12 = 64px]
#define    OLED_SETCONTRAST      0x81 // Set Contrast
#define    OLED_CONTRASTSETTING  0x7F // Contrast Setting 8F / 7F
#define    OLED_SETPRECHGPERIOD  0xD9 // Set Precharge Period(the period during which the pixels of the OLED screen are prepared before being activated)
#define    OLED_PRECHGPERIOD     0xF1 // precharge period = 22 / F1=64px+32px CONTRAST?
#define    OLED_SETVCOMHDESELECT 0xDB // set VCOMh deselect(ensure OLED display pixels receive the correct bias voltage)
#define    OLED_VCOMHDESELECTLVL 0x40 // VCOMh deselect level
#define    OLED_DISABLESCROLL    0x2E // Disable Scrolling(The displayed content will stay fixed on the screen without being continuously moved)
#define    OLED_FULLDISPLAYOFF   0xA4 // Set display to memory content(enable display outputs according to the GDDRAM contents.)
#define    OLED_SETNORMALDISPLAY 0xA6 // set non-inverted display(sets the display to be either normal or inverse)
#define    OLED_DISPLAYON        0xAF // Turn display on


// Function Declarations
void OLED_STM32_initDisplay(void);
void OLED_STM32_sendBuffer(uint8_t *buffer, uint8_t bufferType, uint16_t numberOfElements);
void OLED_STM32_digitalWrite(uint16_t GPIO_Pin, GPIO_PinState PinState);
void OLED_STM32_updateDisplay(void);
void OLED_STM32_clearDisplay(void);
void OLED_STM32_drawPixel(uint8_t x, uint8_t y);
void OLED_STM32_updateMainView(void);
void OLED_STM32_drawMonospaceCharacter(uint8_t xPosOffset, uint8_t yPosOffset, uint8_t myChar);
void OLED_STM32_drawMonospaceString(uint8_t xPos, uint8_t yPos, const char* myString);
uint8_t OLED_STM32_getMonospaceGlyphIndex(uint8_t charIndex);
void OLED_STM32_drawLine(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd);
void OLED_STM32_drawLargeCharacter(uint8_t xPosOffset, uint8_t yPosOffset, uint8_t myChar);
void OLED_STM32_drawLargeString(uint8_t xPos, uint8_t yPos, const char* myString);
uint8_t OLED_STM32_getLargeGlyphIndex(uint8_t charIndex);
void OLED_STM32_drawImage(uint8_t xPosOffset, uint8_t yPosOffset);
