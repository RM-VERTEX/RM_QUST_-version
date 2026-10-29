// [解析] 视觉通信任务：1 kHz 把本车姿态打包成 43 字节发给上位机，同时把上位机回传的 29 字节指令存成快照。
// [解析] 收发是两条独立通道：发送在本任务里，接收在 USB 中断里（usbd_cdc_if.c -> vision_callback -> vis.update）。
// [解析] 频率必须是 1 kHz 左右：视觉侧读线程是忙轮询，上行间隔超过几毫秒就可能被判"掉线"并触发重连。
// [解析] 本任务不直接驱动电机：指令经 vision_cmd 暴露给云台控制任务，由电控决定怎么用（前馈/限幅/保护）。

#include "cmsis_os.h"

#include "io/dbus/dbus.hpp"
#include "io/vision/vision.hpp"
#include "main.h"
#include "tools/mahony/mahony.hpp"

// [解析] imu 在 applications/imu_task.cpp 定义（1 kHz 更新），remote 在 applications/uart_task.cpp 定义
extern sp::Mahony imu;
extern sp::DBus remote;

sp::Vision vis;  // 协议层单例：中断里收、本任务里发

// [解析] 供云台控制任务读取的最新视觉指令；valid == false 时必须当作"没有目标"处理
sp::Vision::Control vision_cmd;

namespace
{
// [解析] 弹速：先用固定值，实车要用裁判系统/测速仪标定后替换
constexpr float VISION_BULLET_SPEED = 24.0f;

// [解析] 累计发弹数：接上发弹检测后改成真实计数，视觉靠它判断"这一枪真的打出去了"
uint16_t g_bullet_count = 0;

// [解析] 操作手档位 -> 协议 mode：右拨杆上=自瞄(1)，中/下=空闲(0)。
// [解析] 若你们的操作习惯不同，只改这一个函数即可；小符/大符(2/3)本车用不到就不发。
uint8_t mode_from_remote(uint32_t now_ms)
{
  if (!remote.is_alive(now_ms)) return 0;
  return (remote.sw_r == sp::DBusSwitchMode::UP) ? 1 : 0;
}

}  // namespace

extern "C" void vision_task(void const * argument)
{
  (void)argument;

  while (true) {
    const uint32_t now_ms = HAL_GetTick();

    // [解析] 视觉掉线时主动上报 mode=0，避免板子一直拿旧目标当有效指令
    uint8_t mode = mode_from_remote(now_ms);
    if (!vis.alive(now_ms)) mode = 0;

    // [解析] 上行 43 字节：姿态四元数(wxyz) + 欧拉角/角速度 + 弹速 + 累计发弹数
    vis.send(
      mode, imu.q, imu.yaw, imu.vyaw, imu.pitch, imu.vpitch, VISION_BULLET_SPEED, g_bullet_count);

    // [解析] 下行 29 字节：取一帧快照给云台任务用（这里只存，不直接控制电机）
    vis.get_control(vision_cmd);

    osDelay(1);  // 1 kHz
  }
}

// [解析] USB CDC 接收回调（中断上下文）：只做解析，绝不做控制/打印/延时
extern "C" void vision_callback(uint8_t * buf, uint32_t len) { vis.update(buf, len); }
