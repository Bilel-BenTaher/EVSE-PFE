/*
 * usart_stm32_console.c
 *
 *  Created on: Jul 19, 2024
 *      Author: hp
 */

#include "stm32u5xx.h"
#include "usart_stm32_console.h"
#include "string.h"
#include <stdlib.h>


void USART_STM32_sendStringToUSART(char const *givenString) {
    // Pointer vers l'instance USART, remplacer USARTx par l'instance USART spécifique
    // Envoyer chaque caractère de la chaîne
    while (*givenString) {
        HAL_UART_Transmit(&huart1, (uint8_t*)givenString, 1, HAL_MAX_DELAY);
        givenString++;
    }

    // Envoyer le caractère de nouvelle ligne (LF)
    uint8_t newline = 0x0A;  // Line Feed
    HAL_UART_Transmit(&huart1, &newline, 1, HAL_MAX_DELAY);

    // Envoyer le retour chariot (CR)
    uint8_t carriageReturn = 0x0D;  // Carriage Return
    HAL_UART_Transmit(&huart1, &carriageReturn, 1, HAL_MAX_DELAY);
}


void USART_STM32_sendIntegerToUSART(char const *givenString, uint16_t myInteger) {
    // Envoi de la chaîne de caractères
    while (*givenString) {
        HAL_UART_Transmit(&huart1, (uint8_t*)givenString, 1, HAL_MAX_DELAY);
        givenString++;
    }

    // Conversion de l'entier en chaîne de caractères
    char buffer[8];
    snprintf(buffer, sizeof(buffer), "%u", myInteger);

    // Envoi de la chaîne représentant l'entier
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
