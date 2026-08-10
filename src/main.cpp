#include "stm32f1xx_hal.h"

int main() {
  HAL_Init();

  while (true) {
    HAL_Delay(5);
  }
}