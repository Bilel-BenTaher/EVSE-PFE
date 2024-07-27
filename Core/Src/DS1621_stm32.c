/*
 * DS1621_stm32.c
 *
 *  Created on: Jul 24, 2024
 *      Author: hp
 */
#include "DS1621_stm32.h"
#include <string.h>
#include <stdio.h>

HAL_StatusTypeDef DS1621_startConversion(I2C_HandleTypeDef *hi2c) {
	uint8_t cmd = 0xEE; // Commande pour démarrer la conversion
	if (HAL_I2C_IsDeviceReady(hi2c, TEMP_ADRESS, 2, HAL_MAX_DELAY) != HAL_OK) {
	        //USART_STM32_sendStringToUSART("Erreur de connexion");
		  Error_Handler();
	    }
    return HAL_I2C_Master_Transmit(hi2c, TEMP_ADRESS, cmd, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef DS1621_readTemperature(I2C_HandleTypeDef *hi2c, float *temperature) {
    uint8_t buf[2];
    uint16_t val;
    HAL_StatusTypeDef retour;
    int i;
    float temp_sum = 0;
    float temp_c;

    for (i = 0; i < DEBOUNCE_READINGS; i++) {
    // Commencer la conversion de température
    retour = DS1621_startConversion(hi2c);
    if (retour != HAL_OK) {
    	//USART_STM32_sendStringToUSART("Erreur de Commencer  ");
    	Error_Handler();
    }
    HAL_Delay(1000); // Attendre que la conversion se termine
    // Demander la lecture de la température
    buf[0] = 0xAA;
    retour = HAL_I2C_Master_Transmit(hi2c, TEMP_ADRESS, buf, 1, HAL_MAX_DELAY);
    if (retour != HAL_OK) {
    	//USART_STM32_sendStringToUSART("Erreur de Lecture ");
    	Error_Handler();
    }
    // Lire les données de température
    retour = HAL_I2C_Master_Receive(hi2c, TEMP_ADRESS, buf, 2, HAL_MAX_DELAY);
    if (retour != HAL_OK) {
    	//USART_STM32_sendStringToUSART("Erreur de receive");
    	Error_Handler();
    }

    // Combinaison des deux octets pour obtenir la valeur de température
    val = ((int16_t)buf[0] << 4) | (buf[1] >> 4);
    // Gestion des températures négatives
    if (val > 0x7FF) {
        val |= 0xF000;
    }
    // Conversion en degré Celsius
     temp_c = val * 0.0625;
     temp_sum += temp_c;
}

    *temperature = temp_sum / DEBOUNCE_READINGS; // Moyenne des lectures

    return HAL_OK;
}
float DS1621_getTemperature(I2C_HandleTypeDef *hi2c) {
    float temperature;
    HAL_StatusTypeDef status = DS1621_readTemperature(hi2c, &temperature);

    if (status == HAL_OK) {
        return temperature;
    } else {
        // Gestion des erreurs : retourne une valeur impossible
        return -999.0;
    }
}
