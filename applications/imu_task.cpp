// [解析] 姿态解算：读 BMI088（SPI1，PA4 选加速度计 / PB0 选陀螺仪），用 Mahony 融合出 yaw/pitch/roll 与四元数。
// [解析] freertos.c 以 osPriorityBelowNormal、栈 256 字创建；1ms 一轮（1000Hz），姿态经 extern 供 vision_task 共用。
// [解析] ★ Mahony 的 dt=1e-3f 必须和 osDelay(1) 对齐，填错积分出的角度会越飘越远。

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

// [解析] 安装方向修正矩阵：把 BMI088 芯片系 {b} 的角速度/加速度转到云台系 {a}（前/左/上）。
// [解析] R_ab 的 3 列分别表示芯片 +x/+y/+z 三根轴在云台系里的坐标（推导见 io/bmi088/readme.md）。
// [解析] ★ 实测依据（2026-11-03，手转验证）：绕云台 pitch 轴转时只有 imu.vpitch 变化（俯头为正、仰头为负），
// [解析]   绕 yaw 轴转只有 imu.vyaw 变化 —— 说明芯片系与云台系**重合**，用单位阵。
// [解析]   （之前一版误判成"横装 90°"：那是把 vroll 看错了。轴映射就用下面这个单位阵，不要再改。）
const float r_ab[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// C板
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);

// 达妙

// sp::BMI088 bmi088(&hspi2, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_3, r_ab);

sp::Mahony imu(1e-3f);

extern "C" void imu_task()
{
  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);

    // 使用调试(f5)查看bmi088和imu内部变量的变化

    osDelay(1);
  }
}
