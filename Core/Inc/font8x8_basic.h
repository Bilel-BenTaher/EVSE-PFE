/*
 * @file font8x8_basic.h
 * @brief Header file for the 8x8 basic monospace font definition.
 *
 * This file provides the declaration of an 8x8 monospace font used for text rendering.
 * It includes the font data and width information for each character in the font set.
 *
 * @date July 8, 2024
 * @author hp
 */

#ifndef INC_FONT8X8_BASIC_H_
#define INC_FONT8X8_BASIC_H_

#include <stdint.h> // Include standard integer types

/**
 * @brief Monospace 8x8 font data array.
 *
 * This array contains the bitmap data for each character in the monospace 8x8 font.
 * Each character is represented as an 8x8 pixel matrix.
 * The font set contains 103 characters.
 */
extern const uint8_t monospaceFont[103][8];

/**
 * @brief Widths of characters in the monospace 8x8 font.
 *
 * This array holds the width of each character in the monospace 8x8 font.
 * Each entry corresponds to the width of the respective character in the `monospaceFont` array.
 * The font set contains 103 characters.
 */
extern const uint8_t monospaceFontWidth[103];

#endif /* INC_FONT8X8_BASIC_H_ */
