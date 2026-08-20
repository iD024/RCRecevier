#pragma once
#include "stm32f1xx_hal.h"

namespace RC::Debug {

class DebugConsole {
public:
  /// Initialize with UART handle (e.g. &huart1)
  static void init(UART_HandleTypeDef *huart);

  /// Print a null-terminated string
  static void print(const char *str);

  /// Print formatted string (max 128 chars)
  static void printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

private:
  static UART_HandleTypeDef *huart_;
};

} // namespace RC::Debug
