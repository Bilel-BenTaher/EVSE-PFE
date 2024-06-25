/*
 * flash_stm32.h
 *
 *  Created on: Jun 25, 2024
 *      Author: hp
 */

#ifndef INC_FLASH_STM32_H_
#define INC_FLASH_STM32_H_
#endif /* INC_FLASH_STM32_H_ */


//* Memories definition */
//MEMORY
//{
 // RAM	   (xrw)	: ORIGIN = 0x20000000,	LENGTH = 768K
  //SRAM4	(xrw)	: ORIGIN = 0x28000000,	LENGTH = 16K
 // FLASH	(rx)	: ORIGIN = 0x08000000,	LENGTH = 1024K
//}

//Variable Definitions
#ifdef STM32U575VGT6
	#define MAXIMUM_AMPERE_ADDRESS 0x08000000  // 1024K STM32U575VGT6
#endif
#define MAXIMUM_AMPERE_ADDRPTR ((uint16_t*) ((uint32_t) MAXIMUM_AMPERE_ADDRESS))


// Function Declarations
void FLASH_STM32_setNewMaximumAmpere(uint8_t newValue);
uint8_t FLASH_STM32_getMaximumAmpere(void);
