#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// C板
sp::DBus remote(&huart3);

// 达妙
// sp::DBus remote(&huart5, false);

// This file is C++ (it instantiates the sp:: namespaces), but the symbols it
// defines are referenced from C: freertos.c declares `uart_task` in its task
// list. extern "C" is therefore required so the names are not mangled.
extern "C" void uart_task(void const * argument)
{
  (void)argument;

  remote.request();

  while (true) {
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}