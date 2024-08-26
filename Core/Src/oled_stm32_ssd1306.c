/*
 * @file oled_stm32_ssd1306.c
 * @brief Source file for STM32 OLED SSD1306 interface functions.
 *
 * This file contains the implementation of functions used to control and update
 * the OLED display based on the SSD1306 driver using the STM32 microcontroller.
 * The functions include display initialization, pixel manipulation, and drawing
 * of characters and strings on the display.
 *
 * @date July 8, 2024
 * @author hp
 */

#include "stm32u5xx.h"
#include "helper_stm32.h"
#include "font8x8_basic.h"
#include <math.h>
#include <stdlib.h>


// Buffer containing initialization commands for the OLED displays
uint8_t OLED_STM32_commandBuffer[COMMAND_BUFFER_LENGTH] = {OLED_DISPLAYOFF, OLED_SETCLOCKDIV, OLED_CLOCKDIVSETTING, OLED_SETMULTIPLEX, OLED_MULTIPLEXSETTING, OLED_SETDISPLAYOFFSET, OLED_DISPLAYOFFSET, OLED_SETSTARTLINE, OLED_CHGPUMPSETTING, OLED_SETCHGPUMP, OLED_SETADDRESSMODE, OLED_HORZPAGEMODE, OLED_SEGMENTREMAP, OLED_SCANDIRECTION, OLED_SETCOMPINS, OLED_COMPINSSETTING, OLED_SETCONTRAST, OLED_CONTRASTSETTING, OLED_SETPRECHGPERIOD, OLED_PRECHGPERIOD, OLED_SETVCOMHDESELECT, OLED_VCOMHDESELECTLVL, OLED_DISABLESCROLL, OLED_FULLDISPLAYOFF, OLED_SETNORMALDISPLAY, OLED_DISPLAYON};

// Buffer containing the display data (each bit represents a pixel)
uint8_t OLED_STM32_displayBuffer[DISPLAY_BUFFER_LENGTH];


/**
 * @brief Initializes the OLED display by sending the initialization commands and clearing the display buffer.
 */
void OLED_STM32_initDisplay(void) {
    // Clear the display buffer
    for (int i = 0; i < DISPLAY_BUFFER_LENGTH; i++) {
        OLED_STM32_displayBuffer[i] = 0;
    }

    // Perform hardware reset on OLED display
    OLED_STM32_digitalWrite(OLED_RST_PIN_Pin, GPIO_PIN_RESET);
    OLED_STM32_digitalWrite(OLED_RST_PIN_Pin, GPIO_PIN_SET);

    // Start communication with the OLED display
    OLED_STM32_digitalWrite(OLED_CS_PIN_Pin, GPIO_PIN_RESET);

    // Send initialization commands and clear the display
    OLED_STM32_sendBuffer(OLED_STM32_commandBuffer, OLED_SPI_COMMAND, COMMAND_BUFFER_LENGTH);
    OLED_STM32_sendBuffer(OLED_STM32_displayBuffer, OLED_SPI_DATA, DISPLAY_BUFFER_LENGTH);

    // End communication with the OLED display
    OLED_STM32_digitalWrite(OLED_CS_PIN_Pin, GPIO_PIN_SET);
}


/**
 * @brief Sends a buffer to the OLED display over SPI.
 *
 * @param buffer Pointer to the buffer to send.
 * @param bufferType Type of buffer (OLED_SPI_COMMAND or OLED_SPI_DATA).
 * @param numberOfElements Number of elements in the buffer to send.
 */
void OLED_STM32_sendBuffer(uint8_t *buffer, uint8_t bufferType, uint16_t numberOfElements) {
    // Set DC pin based on buffer type
    if (bufferType == OLED_SPI_DATA) {
        OLED_STM32_digitalWrite(OLED_DC_PIN_Pin, GPIO_PIN_SET);
    } else {
        OLED_STM32_digitalWrite(OLED_DC_PIN_Pin, GPIO_PIN_RESET);
    }

    // Transmit buffer over SPI
    for (uint16_t i = 0; i < numberOfElements; i++) {
        HAL_SPI_Transmit(&hspi1, &buffer[i], 1, HAL_MAX_DELAY);
        // Wait until transmission is complete
        while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);
    }

    // Wait until SPI is not busy anymore
    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET);

    // Reset DC pin after command transmission
    if (bufferType == OLED_SPI_COMMAND) {
        OLED_STM32_digitalWrite(OLED_DC_PIN_Pin, GPIO_PIN_SET);
    }
}


/**
 * @brief Sets the state of a GPIO pin used for OLED control.
 *
 * @param GPIO_Pin The GPIO pin to set (e.g., OLED_CS_PIN_Pin).
 * @param PinState The state to set the pin (GPIO_PIN_SET or GPIO_PIN_RESET).
 */
void OLED_STM32_digitalWrite(uint16_t GPIO_Pin, GPIO_PinState PinState) {
    // Use different ports based on the pin
    if (GPIO_Pin == OLED_CS_PIN_Pin) {
        HAL_GPIO_WritePin(GPIOB, GPIO_Pin, PinState);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_Pin, PinState);
    }
}


/**
 * @brief Sends the current display buffer to the OLED display, updating the visible content.
 */
void OLED_STM32_updateDisplay(void) {
    OLED_STM32_digitalWrite(OLED_CS_PIN_Pin, GPIO_PIN_RESET);
    OLED_STM32_sendBuffer(OLED_STM32_displayBuffer, OLED_SPI_DATA, DISPLAY_BUFFER_LENGTH);
    OLED_STM32_digitalWrite(OLED_CS_PIN_Pin, GPIO_PIN_SET);
}


/**
 * @brief Clears the entire display buffer.
 *
 * Note: This function only clears the buffer in memory. To apply the changes on the display,
 *       call OLED_STM32_updateDisplay().
 */
void OLED_STM32_clearDisplay(void) {
    for (int i = 0; i < DISPLAY_BUFFER_LENGTH; i++) {
        OLED_STM32_displayBuffer[i] = 0;
    }
}

/**
 * @brief Clears a specific rectangular area of the display buffer.
 *
 * @param x The X coordinate of the top-left corner of the area.
 * @param y The Y coordinate of the top-left corner of the area.
 * @param myString The string whose display area should be cleared.
 */
void OLED_STM32_clearArea(uint8_t x, uint8_t y, const char* myString) {
	// Ensure the coordinates are within the display bounds
    if (x >= OLED_DISPLAY_WIDTH || y >= OLED_DISPLAY_HEIGHT) return;

    uint8_t width = 0;
	uint8_t height = 8;  // Fixed height for monospace font (assuming 8 pixels per character height)

	// Calculate the total width required to clear based on the string length
    while (myString[counter] != '\0') {
        width += monospaceFontWidth[OLED_STM32_getMonospaceGlyphIndex(myString[counter])];
        counter++;
    }

    // Calculate the right and bottom edges of the area to clear
        uint8_t x_end = (x + width > OLED_DISPLAY_WIDTH) ? OLED_DISPLAY_WIDTH : x + width;
        uint8_t y_end = (y + height > OLED_DISPLAY_HEIGHT) ? OLED_DISPLAY_HEIGHT : y + height;

        // Clear the specified area in the display buffer
        for (uint8_t row = y; row < y_end; row++) {
            for (uint8_t col = x; col < x_end; col++) {
                // Calculate the index in the buffer based on the coordinates
                uint16_t index = (row / 8) * OLED_DISPLAY_WIDTH + col;

                // Clear the bit corresponding to the position (col, row)
                OLED_STM32_displayBuffer[index] &= ~(1 << (row % 8));
        }
    }
}


/**
 * @brief Sets a specific pixel in the OLED display buffer as active.
 *
 * This function marks a pixel at the specified (x, y) coordinates as active in the display buffer.
 * The buffer must be sent to the OLED display using another function after all drawing operations are complete.
 *
 * @param x The horizontal coordinate of the pixel.
 * @param y The vertical coordinate of the pixel.
 */
void OLED_STM32_drawPixel(uint8_t x, uint8_t y) {
    // Ensure the coordinates are within the display bounds
    if ((x < OLED_DISPLAY_WIDTH) && (y < OLED_DISPLAY_HEIGHT)) {
        // Calculate the index in the buffer and set the bit corresponding to the pixel (x, y)
        OLED_STM32_displayBuffer[x + (y / 8) * OLED_DISPLAY_WIDTH] |= 1 << (y % 8);
    }
}

/**
 * @brief Draws a monospace character in the OLED display buffer.
 *
 * This function draws an 8x8 pixel character glyph at the specified position in the display buffer.
 * The buffer must be sent to the OLED display using another function after all drawing operations are complete.
 *
 * @param xPosOffset The horizontal position offset in pixels.
 * @param yPosOffset The vertical position offset in pixels.
 * @param myChar The character to draw.
 */
void OLED_STM32_drawMonospaceCharacter(uint8_t xPosOffset, uint8_t yPosOffset, uint8_t myChar) {
	// Iterate over each pixel row (8 rows for an 8x8 character glyph)
	for (int yPos = 0; yPos < 8; yPos++) {
		// Iterate over each pixel column (8 columns for an 8x8 character glyph)
		for (int xPos = 0; xPos < 8; xPos++) {
			// Extract the pixel value from the font array and check if it should be drawn
			uint8_t myValue = (monospaceFont[OLED_STM32_getMonospaceGlyphIndex(myChar)][yPos] & (1<<xPos)) / (pow(2, xPos));
			if (myValue == 1) {
				OLED_STM32_drawPixel(xPos + xPosOffset, yPos + yPosOffset);
			}
		}
	}
}

/**
 * @brief Draws a string of monospace characters in the OLED display buffer.
 *
 * This function draws a string of characters at the specified position in the display buffer.
 * The buffer must be sent to the OLED display using another function after all drawing operations are complete.
 *
 * @param xPos The starting horizontal position in pixels.
 * @param yPos The starting vertical position in pixels.
 * @param myString The string to draw.
 */
void OLED_STM32_drawMonospaceString(uint8_t xPos, uint8_t yPos, const char* myString) {

	uint8_t currentPosX = xPos;  // Track the current X position for the next character

	// Iterate over each character in the string
	int counter = 0;
	while (myString[counter] != '\0') {
		// Draw the character at the current position
		OLED_STM32_drawMonospaceCharacter(currentPosX, yPos, myString[counter]);

        // Advance the X position by the width of the character
		currentPosX += monospaceFontWidth[OLED_STM32_getMonospaceGlyphIndex(myString[counter])];
		counter++;
	}
}

/**
 * @brief Calculates the glyph index for a given character in the monospace font.
 *
 * This function maps characters to their corresponding glyph index in the monospace font.
 * It handles special cases for certain characters and returns the appropriate index.
 *
 * @param charIndex The ASCII value of the character.
 * @return The glyph index corresponding to the character.
 */
uint8_t OLED_STM32_getMonospaceGlyphIndex(uint8_t charIndex) {
    // Map special characters to specific font indices
    switch (charIndex) {
        case 0xC4: return 95;  // Ä
        case 0xE4: return 96;  // ä
        case 0xD6: return 97;  // Ö
        case 0xF6: return 98;  // ö
        case 0xDC: return 99;  // Ü
        case 0xFC: return 100; // ü
        case 0xDF: return 101; // ß
        case 0xF8: return 102; // ø
        default: return charIndex - 32; // Default character mapping (ASCII offset)
    }
}

/**
 * @brief Draws a line on the OLED display.
 *
 * This function draws a vertical or horizontal line between two points on the OLED display.
 * It does not support diagonal lines.
 *
 * @param xStart The x-coordinate of the start point.
 * @param yStart The y-coordinate of the start point.
 * @param xEnd The x-coordinate of the end point.
 * @param yEnd The y-coordinate of the end point.
 */
void OLED_STM32_drawLine(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd) {
	// Ensure that the line is either vertical or horizontal
	    if (xStart == xEnd) {
	        // Vertical line
	        for (int y = yStart; y <= yEnd; y++) {
	            OLED_STM32_drawPixel(xStart, y);
	        }
	    } else if (yStart == yEnd) {
	        // Horizontal line
	        for (int x = xStart; x <= xEnd; x++) {
	            OLED_STM32_drawPixel(x, yStart);
	        }
	    }
	    // No action for diagonal lines
	}

/**
 * @brief Updates the main "Bienvenue" view on the OLED display.
 *
 * This function retrieves the current time and date, clears the display,
 * draws the welcome view, and updates the display with the new content.
 */
void OLED_STM32_updateMain_BienvenueView()
{
   // Retrieve the current time and date
   get_time();
   get_date();

   // Clear the entire display
   OLED_STM32_clearDisplay();

   // Draw lines separating sections of the display
   OLED_STM32_drawLine(0, 9, 127, 9);
   OLED_STM32_drawLine(0, 53, 127, 53);

   // Display the current time, date, and a welcome message
   OLED_STM32_drawMonospaceString(0, 0, Time);
   OLED_STM32_drawMonospaceString(86, 0, date);
   OLED_STM32_drawMonospaceString(38, 32, Bienvenue);

   // Update the OLED display with the new buffer content
   OLED_STM32_updateDisplay();
}


