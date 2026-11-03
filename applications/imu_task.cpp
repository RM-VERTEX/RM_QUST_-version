// [解析] 姿态解算：读 BMI088（SPI1，PA4 选加速度计 / PB0 选陀螺仪），用 Mahony 融合出 yaw/pitch/roll 与四元数。
// [解析] freertos.c 以 osPriorityBelowNormal、栈 256 字创建；1ms 一轮（1000Hz），姿态经 extern 供 vision_task 共用。
// [解析] ★ Mahony 的 dt=1e-3f 必须和 osDelay(1) 对齐，填错积分出的角度会越飘越远。

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

// [解析] 安装方向修正矩阵：BMI088 绕 Z 轴转了 90° 贴板，不修正会把前倾算成右倾；新 X=-Y、新 Y=+X、新 Z=+Z（推导见 io/bmi088/readme.md）。
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
