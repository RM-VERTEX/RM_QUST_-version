#include "cmsis_os.h"
#include "io/adc/adc.hpp"

// C 板：PF10 / ADC3_IN8；
sp::Adc battery(&hadc3, 12, 3.3f, (222.0f / 22.0f), 1.0f);
// 深大飞镖控制板专用配置如下
// F411：PC4 / ADC1_IN14；直接测量引脚电压，分压比 1，补偿 0V。
// sp::Adc battery(&hadc1, 12, 3.3f, 1.0f, 0.0f);
float voltage = 0.0f;

// This symbol is referenced from C (freertos.c task list), so it needs C
// linkage. The parameter list matches the CMSIS-RTOS thread signature.
extern "C" void adc_task(void const * argument)
{
  (void)argument;
  battery.init();
  while (true) {
    if (battery.update()) {
      voltage = battery.voltage;  // 只使用本次成功转换的结果
    }
    osDelay(10);
  }
}