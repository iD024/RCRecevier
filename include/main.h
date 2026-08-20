/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c / main.cpp file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void CubeMX_Init(void);
void SystemClock_Config(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

/* Hardware Handles ----------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* Private defines -----------------------------------------------------------*/
#define Status_LED_Pin GPIO_PIN_13
#define Status_LED_GPIO_Port GPIOC
#define NRF24_CSN_Pin GPIO_PIN_4
#define NRF24_CSN_GPIO_Port GPIOA
#define NRF24_IRQ_Pin GPIO_PIN_0
#define NRF24_IRQ_GPIO_Port GPIOB
#define NRF24_IRQ_EXTI_IRQn EXTI0_IRQn
#define NRF24_CE_Pin GPIO_PIN_11
#define NRF24_CE_GPIO_Port GPIOB
#define Bind_Button_Pin GPIO_PIN_12
#define Bind_Button_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
