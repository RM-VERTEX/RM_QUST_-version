// [解析] 姿态解算：读 BMI088（SPI1，PA4 选加速度计 / PB0 选陀螺仪），用 Mahony 融合出 yaw/pitch/roll 与四元数。
// [解析] freertos.c 以 osPriorityBelowNormal、栈 256 字创建；1ms 一轮（1000Hz），姿态经 extern 供 vision_task 共用。
// [解析] ★ Mahony 的 dt=1e-3f 必须和 osDelay(1) 对齐，填错积分出的角度会越飘越远。

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

// [解析] 安装方向修正矩阵：把 BMI088 芯片坐标系 {b} 的加速度/角速度转到云台坐标系 {a}（前/左/上）。
// [解析] R_ab 的 3 列 = 芯片 +x/+y/+z 三根轴在云台系里的坐标（推导见 io/bmi088/readme.md）。
// [解析] ★ 当前装法：C 板完全正装（长边沿前后、芯片朝上、芯片 +x 朝向云台正前方）→ 两系重合 → 单位阵。
// [解析]   若实测"逆时针转云台 yaw 反而变小"或"抬头 pitch 反而变大"，说明板子实际是整体转了 180°，换成：
// [解析]     {{-1,0,0},{0,-1,0},{0,0,1}}
// [解析]   若板子是芯片朝下贴装的（翻面），换成：
// [解析]     {{1,0,0},{0,-1,0},{0,0,-1}}
// [解析]   旧值 {{0,-1,0},{1,0,0},{0,0,1}} 对应的是"C 板横着装、CAN 口朝前"（绕 Z 转 90°），正装后不再适用。
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
