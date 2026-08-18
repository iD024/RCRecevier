#include "cube_init.h"
#include "stm32f1xx_hal.h"

int main() {
  CubeMX_Init();

  while (true) {
    HAL_Delay(5);
  }
}