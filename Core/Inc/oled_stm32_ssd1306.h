/*
 * oled_stm32_ssd1306.h
 *
 *  Created on: Jul 8, 2024
 *      Author: Bilel BENTAHER
 *
 *  Description:
 *  This header file provides the necessary definitions and function prototypes
 *  for interfacing an OLED display based on the SSD1306 controller with an STM32
 *  microcontroller, specifically the STM32U575VGT6. This includes initialization,
 *  display update, and basic drawing operations.
 */

#ifndef INC_OLED_STM32_SSD1306_H_
#define INC_OLED_STM32_SSD1306_H_

// SPI Command/Date selection for OLED display
#define OLED_SPI_COMMAND      0x00   // Command mode for SPI communication
#define OLED_SPI_DATA         0x01   // Data mode for SPI communication

// Display dimensions in pixels
#define OLED_DISPLAY_WIDTH    128
#define OLED_DISPLAY_HEIGHT   64

// Buffer sizes
#define COMMAND_BUFFER_LENGTH 26   // Buffer length for command sequences
#define DISPLAY_BUFFER_LENGTH (OLED_DISPLAY_WIDTH * OLED_DISPLAY_HEIGHT / 8) // Buffer length for display data

// OLED Command Definitions for SSD1306
#define OLED_DISPLAYOFF       0xAE   // Turn off the OLED display
#define OLED_SETCLOCKDIV      0xD5   // Set the display clock divide ratio/oscillator frequency
#define OLED_CLOCKDIVSETTING  0x80   // Default clock divide ratio setting
#define OLED_SETMULTIPLEX     0xA8   // Set multiplex ratio (1 to 64)
#define OLED_MULTIPLEXSETTING 0x3F   // Default multiplex setting for 64MUX
#define OLED_SETDISPLAYOFFSET 0xD3   // Set vertical display offset
#define OLED_DISPLAYOFFSET    0x00   // No offset by default
#define OLED_SETSTARTLINE     0x40   // Set start line address to 0 (first line)
#define OLED_CHGPUMPSETTING   0x8D   // Set charge pump setting
#define OLED_SETCHGPUMP       0x14   // Enable charge pump
#define OLED_SETADDRESSMODE   0x20   // Set memory addressing mode
#define OLED_HORZPAGEMODE     0x00   // Horizontal addressing mode
#define OLED_SEGMENTREMAP     0xA1   // Set segment re-map to 127->0
#define OLED_SCANDIRECTION    0xC8   // Set COM output scan direction remapped
#define OLED_SETCOMPINS       0xDA   // Set COM pins hardware configuration
#define OLED_COMPINSSETTING   0x12   // Default COM pins configuration
#define OLED_SETCONTRAST      0x81   // Set contrast control
#define OLED_CONTRASTSETTING  0x7F   // Default contrast setting
#define OLED_SETPRECHGPERIOD  0xD9   // Set pre-charge period
#define OLED_PRECHGPERIOD     0xF1   // Default pre-charge period
#define OLED_SETVCOMHDESELECT 0xDB   // Set VCOMH deselect level
#define OLED_VCOMHDESELECTLVL 0x40   // Default VCOMH deselect level
#define OLED_DISABLESCROLL    0x2E   // Disable scrolling
#define OLED_FULLDISPLAYOFF   0xA4   // Entire display OFF (resume to RAM content display)
#define OLED_SETNORMALDISPLAY 0xA6   // Set normal display (non-inverted mode)
#define OLED_DISPLAYON        0xAF   // Turn on the OLED display

// Function Declarations

/**
 * @brief Initializes the OLED display with the specified settings.
 */
void OLED_STM32_initDisplay(void);

/**
 * @brief Sends a buffer of data or commands to the OLED display.
 *
 * @param buffer Pointer to the data/command buffer.
 * @param bufferType Type of buffer being sent (OLED_SPI_COMMAND or OLED_SPI_DATA).
 * @param numberOfElements Number of elements in the buffer to send.
 */
void OLED_STM32_sendBuffer(uint8_t *buffer, uint8_t bufferType, uint16_t numberOfElements);

/**
 * @brief Controls the digital output state of a GPIO pin.
 *
 * @param GPIO_Pin The GPIO pin number to control.
 * @param PinState The desired state of the GPIO pin (HIGH or LOW).
 */
void OLED_STM32_digitalWrite(uint16_t GPIO_Pin, GPIO_PinState PinState);

/**
 * @brief Refreshes the display with the contents of the display buffer.
 */
void OLED_STM32_updateDisplay(void);

/**
 * @brief Clears the entire OLED display by filling the display buffer with zeros.
 */
void OLED_STM32_clearDisplay(void);

/**
 * @brief Clears a specific rectangular area of the display buffer.
 *
 * @param x The X coordinate of the top-left corner of the area.
 * @param y The Y coordinate of the top-left corner of the area.
 * @param myString The string whose display area should be cleared.
 */
void OLED_STM32_clearArea(uint8_t x, uint8_t y, const char* myString);

/**
 * @brief Draws a single pixel at the specified coordinates on the OLED display.
 *
 * @param x The X coordinate of the pixel.
 * @param y The Y coordinate of the pixel.
 */
void OLED_STM32_drawPixel(uint8_t x, uint8_t y);

/**
 * @brief Draws a monospace character at the specified position on the OLED display.
 *
 * @param xPosOffset X position offset for the character.
 * @param yPosOffset Y position offset for the character.
 * @param myChar The character to draw.
 */
void OLED_STM32_drawMonospaceCharacter(uint8_t xPosOffset, uint8_t yPosOffset, uint8_t myChar);

/**
 * @brief Draws a string of monospace characters at the specified position on the OLED display.
 *
 * @param xPos X position for the first character.
 * @param yPos Y position for the first character.
 * @param myString Pointer to the null-terminated string to draw.
 */
void OLED_STM32_drawMonospaceString(uint8_t xPos, uint8_t yPos, const char* myString);

/**
 * @brief Returns the index of the glyph corresponding to the given character.
 *
 * @param charIndex The character index (ASCII value).
 * @return The glyph index for the character.
 */
uint8_t OLED_STM32_getMonospaceGlyphIndex(uint8_t charIndex);

/**
 * @brief Draws a line from (xStart, yStart) to (xEnd, yEnd) on the OLED display.
 *
 * @param xStart X coordinate of the start point.
 * @param yStart Y coordinate of the start point.
 * @param xEnd X coordinate of the end point.
 * @param yEnd Y coordinate of the end point.
 */
void OLED_STM32_drawLine(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd);

/**
 * @brief Updates the display with a "Welcome" view.
 */
void OLED_STM32_updateMain_WelcomeView(void);

#endif /* INC_OLED_STM32_SSD1306_H_ */
