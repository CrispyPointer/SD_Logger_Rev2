#ifndef _GPIO_CONFIG_H_
#define _GPIO_CONFIG_H_

#include "main.h"

/* USB module GPIO */
#define USB_ON HAL_GPIO_ReadPin(Vbus_Sense_GPIO_Port, Vbus_Sense_Pin)

/* SPI SD GPIO */
#define SD_PWR_ON  HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, SET);
#define SD_PWR_OFF HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, RESET);
#define SPI_SD_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15 | SD_CS_Pin, RESET);

/* SPI FRAM GPIO */
#define FRAM_PWR_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | FRAM_CS_Pin | LED_Pin | FRAM_HOLD_Pin, RESET);

/* LED */
#define LED_ON     HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
#define LED_OFF    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
#define LED_TOGGLE HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

#endif // !_GPIO_CONFIG_H_
