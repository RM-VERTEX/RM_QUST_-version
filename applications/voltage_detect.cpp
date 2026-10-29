// [解析] 电池电压检测 + 低压报警：ADC 采分压后的电池电压，低于 24V 时蜂鸣器持续鸣响。
// [解析] C 板参数 = &hadc3 / 12 位(0~4095) / 3.3V 参考 / 分压比 222÷22≈10.09 / 1.0V 补偿；每 10ms 采一次（100Hz）。
// [解析] 报警策略：低于阈值持续 1s 才响（去抖），回到 阈值+0.3V 以上持续 1s 才解除（回差），避免电压在阈值附近抖。
// [解析] 上电后 2s 内不判：等 ADC 和电池电压稳定，也避开 buzzer_task 的开机三声。
// [解析] ★ 阈值是否准，取决于下面 VOLTAGE_CAL_GAIN 的标定，见文件末尾说明。

#include "cmsis_os.h"
#include "io/adc/adc.hpp"
#include "io/buzzer/buzzer.hpp"

// [解析] buzzer 定义在 buzzer_task.cpp（同一个 TIM4_CH3 / PD14），这里只借用、不重复建对象。
extern sp::Buzzer buzzer;

// C 板：PF10 / ADC3_IN8；
// [解析] 偏移量用 0：ADC 通路本身是过零线性的，加性偏移只是当年为了补"3 cycles 采样偏低"做的补偿，
// [解析]   采样时间已经改成 480 cycles，这个补偿就过量了（实测偏高 0.4V）。归零后偏差变成纯比例，一次标定就能全量程准。
sp::Adc battery(&hadc3, 12, 3.3f, (222.0f / 22.0f), 0.0f);

// 标定后的电池电压, 单位 V。调试器里看这个值。
float voltage = 0.0f;

namespace
{
/*==================== 报警参数 ====================*/
constexpr float kLowVoltageV = 24.0f;      // 低压阈值, 单位 V
constexpr float kHysteresisV = 0.3f;       // 回差: 高于 24.3V 才算恢复
constexpr uint32_t kDebounceMs = 1000;     // 低于/高于阈值都要持续这么久才翻转状态
constexpr uint32_t kStartupGraceMs = 2000; // 上电后这段时间内不报警

/**
 * 标定增益 = 万用表实测电压 / 本模块读数（偏移已归零，所以对全量程都成立）。
 *
 * 本次标定实测：万用表 24.1V 时，去掉 1.0V 偏移后的读数是 23.5V
 *   → 1.0255 = 24.1 / 23.5
 * 等价的物理含义：有效分压比 ≈ 10.09 × 1.0255 ≈ 10.35（不是标称的 222/22）。
 *
 * 换板子/换分压电阻后重新标：万用表量 V_real，看全局 voltage，令 GAIN = V_real / voltage。
 */
constexpr float kVoltageCalGain = 1.0255f;

/*==================== 报警音（与 buzzer_task 一致）====================*/
constexpr float kBeepHz = 5000.0f;
constexpr float kBeepDuty = 0.1f;

/*==================== 状态 ====================*/
bool g_low_latched = false;       // 已判定为低压（对外）
bool g_pending = false;           // 正在累计的候选状态
uint32_t g_pending_since = 0;     // 候选起始时刻
bool g_buzzer_on = false;         // 蜂鸣器硬件当前状态
bool g_has_value = false;         // 是否至少成功采到过一次
uint32_t g_start_ms = 0;          // 任务启动时刻

// [解析] 分压源内阻高（100k∥10k≈9kΩ，222k∥22k≈20kΩ），CubeMX 生成的 3 cycles 采样时间不够，
// [解析]   采样电容建立不满，读数会系统性偏低。这里强制改成 480 cycles（21MHz 下约 23us），
// [解析]   写在代码里，CubeMX 重新生成代码也不会被覆盖。
void apply_sampling_time()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1U;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfig.Offset = 0U;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

// [解析] 低压判定：去抖 + 回差，状态一旦翻转就稳定，不会在阈值附近来回响。
// [解析] 返回 true 表示现在应该鸣响。
bool low_voltage_alarm(uint32_t now)
{
  if ((now - g_start_ms) < kStartupGraceMs) {
    g_pending = false;
    return false;
  }

  // [解析] 一次都没采成功时不做判断：否则 voltage 还是初值 0，会被当成"电压过低"一直响。
  if (!g_has_value) {
    g_pending = false;
    return false;
  }

  const float threshold = g_low_latched ? (kLowVoltageV + kHysteresisV) : kLowVoltageV;
  const bool cross = g_low_latched ? (voltage > threshold) : (voltage < threshold);

  if (!cross) {
    g_pending = false;  // 回到阈值另一侧, 重新计时
  }
  else if (!g_pending) {
    g_pending = true;
    g_pending_since = now;
  }
  else if ((now - g_pending_since) >= kDebounceMs) {
    g_low_latched = !g_low_latched;
    g_pending = false;
  }

  return g_low_latched;
}

// [解析] 只写硬件一次：状态没变就不重复动寄存器（PWM 起停有开销，也没必要）。
void buzzer_set(bool on)
{
  if (on == g_buzzer_on) return;

  if (on) {
    buzzer.set(kBeepHz, kBeepDuty);
    buzzer.start();
  }
  else {
    buzzer.stop();
  }
  g_buzzer_on = on;
}
}  // namespace

extern "C" void adc_task(void * argument)
{
  (void)argument;

  battery.init();        // 改分频 + 算换算系数
  apply_sampling_time(); // 必须在 init() 之后：HAL_ADC_Init 可能会把通道采样时间复位
  g_start_ms = osKernelSysTick();

  while (true) {
    if (battery.update()) {
      voltage = battery.voltage * kVoltageCalGain;  // 只使用本次成功转换的结果
      g_has_value = true;
    }

    buzzer_set(low_voltage_alarm(osKernelSysTick()));
    osDelay(10);
  }
}

/*==============================================================================
 * 标定方法（只做一次，1 分钟）
 *
 * 1) 上电，调试器里看全局变量 `voltage`；
 * 2) 万用表量电池真实电压 V_real；
 * 3) 令 kVoltageCalGain = V_real / voltage，重新编译。
 *
 * 本板已完成一次标定（见 kVoltageCalGain 的注释），除非换分压电阻，不必重做。
 *
 * 常见现象与处理：
 * - 读数偏差是"等比例"的（24V 差 0.4V、26V 差 0.43V）→ 正常，增益一次修好全量程；
 *   若偏差是"固定值"（各电压都差同一个数）→ 才需要动 sp::Adc 的偏移量（现在为 0）。
 * - 读数跳动大 → 检查分压电阻是否虚焊；采样时间已由 apply_sampling_time() 提到 480 cycles。
 * - 想要周期性"嘀嘀"而不是长鸣 → 把 buzzer_set() 的 on 改成按时间取模即可。
 *============================================================================*/
