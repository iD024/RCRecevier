#include "core/debug.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace RC::Debug {

UART_HandleTypeDef *DebugConsole::huart_ = nullptr;

void DebugConsole::init(UART_HandleTypeDef *huart) { huart_ = huart; }

void DebugConsole::print(const char *str) {
  if (huart_ == nullptr || str == nullptr) {
    return;
  }
  HAL_UART_Transmit(huart_, reinterpret_cast<const uint8_t *>(str),
                    std::strlen(str), 100U);
}

void DebugConsole::printf(const char *format, ...) {
  if (huart_ == nullptr || format == nullptr) {
    return;
  }
  char buf[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  print(buf);
}

} // namespace RC::Debug
