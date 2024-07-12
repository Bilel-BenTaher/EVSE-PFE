/*
 * oled_stm32_ssd1306.c
 *
 *  Created on: Jul 8, 2024
 *      Author: hp
 */
#include "stm32u5xx.h"
#include "oled_stm32_ssd1306.h"
#include "helper_stm32.h"
#include "font8x8_basic.h"
#include <math.h>
#include <stdlib.h>


// Variable Declarations
uint8_t OLED_STM32_commandBuffer[COMMAND_BUFFER_LENGTH] = {OLED_DISPLAYOFF, OLED_SETCLOCKDIV, OLED_CLOCKDIVSETTING, OLED_SETMULTIPLEX, OLED_MULTIPLEXSETTING, OLED_SETDISPLAYOFFSET, OLED_DISPLAYOFFSET, OLED_SETSTARTLINE, OLED_CHGPUMPSETTING, OLED_SETCHGPUMP, OLED_SETADDRESSMODE, OLED_HORZPAGEMODE, OLED_SEGMENTREMAP, OLED_SCANDIRECTION, OLED_SETCOMPINS, OLED_COMPINSSETTING, OLED_SETCONTRAST, OLED_CONTRASTSETTING, OLED_SETPRECHGPERIOD, OLED_PRECHGPERIOD, OLED_SETVCOMHDESELECT, OLED_VCOMHDESELECTLVL, OLED_DISABLESCROLL, OLED_FULLDISPLAYOFF, OLED_SETNORMALDISPLAY, OLED_DISPLAYON};
uint8_t OLED_STM32_displayBuffer[DISPLAY_BUFFER_LENGTH];

// Execute initialization commands and configure display with initial command and display buffer.
void OLED_STM32_initDisplay(void) {

	for (int i = 0; i < DISPLAY_BUFFER_LENGTH; i++) { OLED_STM32_displayBuffer[i] = 0; }
	OLED_STM32_digitalWrite(OLED_RST_PIN, GPIO_PIN_RESET);
	OLED_STM32_digitalWrite(OLED_RST_PIN, GPIO_PIN_SET);
	OLED_STM32_digitalWrite(OLED_CS_PIN, GPIO_PIN_RESET);
	OLED_STM32_sendBuffer(OLED_STM32_commandBuffer, OLED_SPI_COMMAND, COMMAND_BUFFER_LENGTH);
	OLED_STM32_sendBuffer(OLED_STM32_displayBuffer, OLED_SPI_DATA, DISPLAY_BUFFER_LENGTH);
	OLED_STM32_drawMonospaceString(38,28, Bienvenue);
	OLED_STM32_drawMonospaceString(20,56,"A");
	OLED_STM32_drawMonospaceString(46,56,"Saisir");
	OLED_STM32_drawMonospaceString(105,56,"M");
	OLED_STM32_digitalWrite(OLED_CS_PIN, GPIO_PIN_SET);

}


void OLED_STM32_sendBuffer(uint8_t *buffer, uint8_t bufferType, uint16_t numberOfElements) {
    if (bufferType == OLED_SPI_DATA) {
        OLED_STM32_digitalWrite(OLED_DC_PIN, GPIO_PIN_SET);
    } else {
        OLED_STM32_digitalWrite(OLED_DC_PIN, GPIO_PIN_RESET);
    }

    for (uint16_t i = 0; i < numberOfElements; i++) {
        HAL_SPI_Transmit(&hspi1, &buffer[i], 1, HAL_MAX_DELAY);
        while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET); // Wait until transmit complete
    }

    while (__HAL_SPI_GET_FLAG(&hspiX, SPI_FLAG_BSY) != RESET); // Wait until SPI is not busy anymore

    if (bufferType == OLED_SPI_COMMAND) {
        OLED_STM32_digitalWrite(OLED_DC_PIN, GPIO_PIN_SET);
    }
}

// Helper function for pulling OLED pins high or low.
// Example: OLED_STM32_digitalWrite(OLED_DC_PIN, HIGH);
    void OLED_STM32_digitalWrite(uint16_t GPIO_Pin, GPIO_PinState PinState) {
        if (GPIO_Pin == OLED_CS_PIN) {
            HAL_GPIO_WritePin(GPIOB, GPIO_Pin, PinState);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_Pin, PinState);
        }
    }


// This function send the current array of the OLED buffer to the device over SPI.
void OLED_STM32_updateDisplay(void) {

	OLED_STM32_digitalWrite(OLED_CS_PIN,GPIO_PIN_RESET);
	OLED_STM32_sendBuffer(OLED_STM32_displayBuffer, OLED_SPI_DATA, DISPLAY_BUFFER_LENGTH);
	OLED_STM32_digitalWrite(OLED_CS_PIN, GPIO_PIN_SET);

}


// This function clears the array of the OLED buffer.
// The OLED buffer needs to be sent when drawing is completed by another call.
void OLED_STM32_clearDisplay(void) {

	for (int i = 0; i < DISPLAY_BUFFER_LENGTH; i++) { OLED_STM32_displayBuffer[i] = 0; }

}


// This function can place an individual pixel as active into the array of the OLED buffer.
// The OLED buffer needs to be sent when drawing is completed by another call.
void OLED_STM32_drawPixel(uint8_t x, uint8_t y) {

	if ((x < OLED_DISPLAY_WIDTH) && (y < OLED_DISPLAY_HEIGHT)) {
		OLED_STM32_displayBuffer[x + (y / 8) * OLED_DISPLAY_WIDTH] |= 1 << (y % 8);
	}

}


// This function will update the display to represent the main view of the system.
void OLED_STM32_updateMainView(void) {

	// Basic Layout Setup
	OLED_STM32_clearDisplay();
	OLED_STM32_drawLine(0,9,127,9);
	OLED_STM32_drawLine(0,53,127,53);
	OLED_STM32_drawMonospaceString(20,56,"A");
	OLED_STM32_drawMonospaceString(46,56,"Saisir");

	// Maximum Ampere View
	char maxAmpStr[4] = "   ";
	if (HELPER_STM32_getMaximumAmpere() < 10) {
		maxAmpStr[0] = HELPER_STM32_getMaximumAmpere() + 48;
		maxAmpStr[1] = 0x41; //A
	} else {
		maxAmpStr[0] = HELPER_STM32_getMaximumAmpere() / 10 + 48;
		maxAmpStr[1] = HELPER_STM32_getMaximumAmpere() % 10 + 48;
		maxAmpStr[2] = 0x41; //A
	}
	OLED_STM32_drawMonospaceString(0,0,maxAmpStr);

	// Temperature View
	char maxTempStr[6] = "     ";
	int8_t currentTemp = HELPER_STM32_getCurrentTemp();
	uint8_t offsetValue = 0;
	// Check if temperature is negative
	if ( currentTemp < 0) {
		maxTempStr[0] = 0x2D; // -
		offsetValue = 1;
		currentTemp = currentTemp * -1;
	}
	// check if temperature is double digit
	if ((currentTemp > 9) | (currentTemp < -9)) {
		maxTempStr[0+offsetValue] = currentTemp / 10 + 48;
		maxTempStr[1+offsetValue] = currentTemp % 10 + 48;
		offsetValue++;
	} else {
		maxTempStr[0+offsetValue] = currentTemp + 48;
	}
	// Add the degree symbols
	maxTempStr[1+offsetValue] = 0xF8; // °
	maxTempStr[2+offsetValue] = 0x43; // C
	OLED_STM32_drawMonospaceString(24, 0, maxTempStr);

	// Status View
	//offsetValue = 0;
	switch (HELPER_STM32_getCurrentStatus()) {
		case DISCONNECTED: offsetValue = 37; OLED_STM32_drawMonospaceString(48+offsetValue, 0, "Séparé"); break;
		case CONNECTED_NO_PWM: offsetValue = 29; OLED_STM32_drawMonospaceString(48+offsetValue,0,"Attachement"); break;
		case CONNECTED: offsetValue = 29; OLED_STM32_drawMonospaceString(48+offsetValue,0,"Connexion"); break;
		case CHARGING: offsetValue = 16; OLED_STM32_drawMonospaceString(48+offsetValue,0,"Chargement"); break;
		case CHARGING_COOLED: offsetValue = 44; OLED_STM32_drawMonospaceString(48+offsetValue,0,"Ventilation"); break;
		case FAULT: offsetValue = 11; OLED_STM32_drawMonospaceString(48+offsetValue,0,"Erreur"); break;
	}

	// Large current Ampere View
	char currentAmpStr[3] = "  ";
	if (HELPER_STM32_getCurrentAmpere() < 10) {
		currentAmpStr[0] = HELPER_STM32_getCurrentAmpere() + 48;
	} else {
		currentAmpStr[0] = HELPER_STM32_getCurrentAmpere() / 10 + 48;
		currentAmpStr[1] = HELPER_STM32_getCurrentAmpere() % 10 + 48;
	}
	if (HELPER_STM32_getCurrentAmpere() < 10) {
		offsetValue = 20;
	} else {
		offsetValue = 0;
	}
	OLED_STM32_drawLargeString(32+offsetValue,18,currentAmpStr);
	OLED_STM32_drawLargeString(76,18,"A");
	OLED_STM32_updateDisplay();

}

// This function is drawing each 8px tall character glyph into the array of the OLED buffer.
// The OLED buffer needs to be sent when drawing is completed by another call.
void OLED_STM32_drawMonospaceCharacter(uint8_t xPosOffset, uint8_t yPosOffset, uint8_t myChar) {

	for (int yPos = 0; yPos < 8; yPos++) {
		for (int xPos = 0; xPos < 8; xPos++) {
			uint8_t myValue = (monospaceFont[OLED_STM32_getMonospaceGlyphIndex(myChar)][yPos] & (1<<xPos)) / (pow(2, xPos));
			if (myValue == 1) { OLED_STM32_drawPixel(xPos + xPosOffset, yPos + yPosOffset); }
		}
	}

}


// This function takes the given string, checks for non-'\n' characters and advances the x-Position by the width of the glyph.
// When the full string is drawn, the updated OLED buffer is sent to the device.
void OLED_STM32_drawMonospaceString(uint8_t xPos, uint8_t yPos, const char* myString) {

	int counter = 0;
	uint8_t currentPosX = xPos;
	while (myString[counter] != 0) {
		OLED_STM32_drawMonospaceCharacter(currentPosX, yPos, myString[counter]);
		currentPosX += monospaceFontWidth[OLED_STM32_getMonospaceGlyphIndex(myString[counter])];
		counter++;
	}
}

// This is a helper function to calcualte the correct font glyph index for any given character
uint8_t OLED_STM32_getMonospaceGlyphIndex(uint8_t charIndex) {

	switch (charIndex) {
		case 0xC4: return 95;
		case 0xE4: return 96;
		case 0xD6: return 97;
		case 0xF6: return 98;
		case 0xDC: return 99;
		case 0xFC: return 100;
		case 0xDF: return 101;
		case 0xF8: return 102;
		default: return charIndex - 32;
	}

}

// This function can currently only draw vertical or horizontal lines. No diagonal lines.
void OLED_STM32_drawLine(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd) {

	// Do something to create a line. Diagonal is tricky.
	for (int x = 0; x <= (xEnd - xStart); x++) {
		for (int y = 0; y <= (yEnd - yStart); y++) {
			OLED_STM32_drawPixel(xStart + x, yStart + y);
		}
	}

}


// This function is drawing a larger font glyph into the array of the OLED buffer.
// This can only be used for characters 0-9, ' ' and 'A'.
// The OLED buffer needs to be sent when drawing is completed by another call.
void OLED_STM32_drawLargeCharacter(uint8_t xPosOffset, uint8_t yPosOffset, uint8_t myChar) {

	uint8_t glyphWidth = latoGlyphWidth[OLED_STM32_getLargeGlyphIndex(myChar)];
    const uint8_t *imageBits = latoFontBits[OLED_STM32_getLargeGlyphIndex(myChar)];
	uint8_t correctionFactor = 0;
	if (latoFontWidth % 8 > 0) { correctionFactor = 1; }
	for (int yPos = 0; yPos < latoFontHeight; yPos++) {
		for (int xPos = 0; xPos < glyphWidth; xPos++) {
			uint16_t currentBit = xPos / 8 + (yPos * (latoFontWidth / 8 + correctionFactor));
			uint8_t currentBitMask = 1 << xPos % 8;
			uint8_t myValue = (imageBits[currentBit] & currentBitMask) / currentBitMask;
			if (myValue) { OLED_STM32_drawPixel(xPos + xPosOffset + latoGlyphOffset[OLED_STM32_getLargeGlyphIndex(myChar)], yPos + yPosOffset); }
		}
	}

}


// This function takes the given string, checks for non-'\n' characters and advances the x-Position by the width of the glyph.
// When the full string is drawn, the updated OLED buffer is sent to the device.
void OLED_STM32_drawLargeString(uint8_t xPos, uint8_t yPos, const char* myString) {

	int counter = 0;
	uint8_t currentPosX = xPos;
	while (myString[counter] != 0) {
		OLED_STM32_drawLargeCharacter(currentPosX, yPos, myString[counter]);
		currentPosX += latoGlyphWidth[OLED_STM32_getLargeGlyphIndex(myString[counter])] + latoGlyphOffset[OLED_STM32_getLargeGlyphIndex(myString[counter])];
		counter++;
	}

}

// This is a helper function to calculate the correct font glyph index for 0-9, ' ' and 'A'
uint8_t OLED_STM32_getLargeGlyphIndex(uint8_t charIndex) {

	switch (charIndex) {
        case 32: return 0;
		case 65: return 11;
		default: return charIndex - 47;
	}
}

/*void OLED_STM32_drawImage(uint8_t xPosOffset, uint8_t yPosOffset) {

	uint8_t correctionFactor = 0;
	if (imageWidth % 8 > 0) { correctionFactor = 1; }
	for (int yPos = 0; yPos < imageHeight; yPos++) {
		for (int xPos = 0; xPos < imageWidth; xPos++) {
			uint16_t currentBit = xPos / 8 + (yPos * (imageWidth / 8 + correctionFactor));
			uint8_t currentBitMask = 1 << xPos % 8;
			uint8_t myValue = (imageBits[currentBit] & currentBitMask) / currentBitMask;
			if (myValue) { OLED_STM32_drawPixel(xPos + xPosOffset, yPos + yPosOffset); }
		}
	}

}
*/
