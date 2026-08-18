#pragma once

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void CubeMX_Init(void);

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#ifdef __cplusplus
}
#endif