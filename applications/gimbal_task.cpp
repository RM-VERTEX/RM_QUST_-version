// 云台控制：串级（位置环 + IMU 角速度环）+ 前馈 + 电压输出。
// Yaw = GM6020 ID1(0x205), Pitch = ID2(0x206)，CAN1，电压帧 0x1FF。
// 角速度用四元数把陀螺投影到关节轴，避开欧拉角在 pitch=±90° 的 1/cos 奇异。

#include <cmath>

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;
extern sp::Mahony imu;
extern sp::BMI088 bmi088;

sp::RM_Motor gimbal_yaw(1, sp::RM_Motors::GM6020_V, 1.0f);
sp::RM_Motor gimbal_pitch(2, sp::RM_Motors::GM6020_V, 1.0f);

//==================== Live Watch ====================
float gimbal_yaw_out_v = 0.0f;
float gimbal_pitch_out_v = 0.0f;
float gimbal_yaw_stick = 0.0f;
float gimbal_pitch_stick = 0.0f;
float gimbal_yaw_angle_deg = 0.0f;
float gimbal_pitch_angle_deg = 0.0f;
float gimbal_yaw_target_deg = 0.0f;
float gimbal_pitch_target_deg = 0.0f;
float gimbal_yaw_pos_err = 0.0f;
float gimbal_pitch_pos_err = 0.0f;
float gimbal_yaw_rate_cmd = 0.0f;
float gimbal_pitch_rate_cmd = 0.0f;
float gimbal_yaw_rate = 0.0f;
float gimbal_pitch_rate = 0.0f;
float gimbal_yaw_rate_raw = 0.0f;
float gimbal_pitch_rate_raw = 0.0f;
float gimbal_yaw_rate_euler = 0.0f;
float gimbal_pitch_rate_euler = 0.0f;
float gimbal_yaw_rate_err = 0.0f;
float gimbal_pitch_rate_err = 0.0f;
float gimbal_pitch_geom_deg = 0.0f;

//==================== 诊断 ====================
uint32_t g_gimbal_loop_hz = 0;
uint32_t g_tx_drop_count = 0;
bool g_imu_stale = false;
bool g_rate_runaway = false;
bool g_pitch_limit_tripped = false;
uint32_t g_can_err_count = 0;
uint32_t g_can_busoff_count = 0;
uint32_t g_can_tec = 0;
uint32_t g_can_rec = 0;
uint32_t g_can_lec = 0;

//==================== 飞行记录仪 ====================
constexpr uint32_t kRecLen = 512;
struct RecSample
{
  float t_ms;
  float pitch_target_deg;
  float pitch_angle_deg;
  float pitch_rate_cmd;
  float pitch_rate;
  float pitch_out_v;
  float yaw_rate_cmd;
  float yaw_rate;
};
RecSample rec_buf[kRecLen];
volatile uint32_t g_rec_state = 0;
uint32_t g_rec_n = 0;
float g_rec_tick = 0.0f;
uint32_t g_rec_reset_req = 0;

//==================== 模式 ====================
// 左拨码：上=自瞄（位控） 中=操作手（速控） 下=断电
constexpr uint8_t kModeAutoAim = 0;
constexpr uint8_t kModeOperator = 1;
constexpr uint8_t kModeOff = 2;
uint8_t g_mode = kModeOff;
uint8_t g_mode_last = 0xFF;

namespace
{
//==================== 参数 ====================
constexpr float kDt = 1e-3f;

// 转速上限
constexpr float kYawMaxRate = 4.0f;
constexpr float kPitchMaxRate = 1.5f;

// 位置环（输入弧度、输出 rad/s）
constexpr float kYawPosKp = 4.0f;
constexpr float kPitchPosKp = 3.0f;
constexpr float kPosDeadzoneDeg = 1.0f;

// 速度环
constexpr float kYawRateKp = 3.2f;
constexpr float kPitchRateKp = 0.4f;
constexpr float kYawRateKi = 0.0f;
constexpr float kPitchRateKi = 0.5f;
constexpr float kPitchRateMaxIout = 3.0f;

// 前馈：V / (rad/s)，实测 0.85V ↔ 0.5rad/s
constexpr float kYawRateFF = 1.7f;
constexpr float kPitchRateFF = 1.7f;

// 电压上限
constexpr float kYawMaxVoltage = 14.0f;
constexpr float kPitchMaxVoltage = 8.0f;

// 死区（位置环负责"不追静摩擦"，速度环几乎不给）
constexpr float kYawRateDeadzone = 0.01f;
constexpr float kPitchRateDeadzone = 0.01f;
constexpr float kIntegralPause = 1.5f;

// 反馈低通：alpha 越大越平滑、滞后越大
constexpr float kRateAlpha = 0.5f;
constexpr float kRateMeasLimit = 20.0f;

// 摇杆
constexpr float kStickDeadzone = 0.06f;
constexpr float kYawLeverDegPerS = 240.0f;
constexpr float kPitchLeverDegPerS = 150.0f;

// 操作手速控模式的灵敏度：摇杆满偏对应的角速度
constexpr float kYawRateSense = 5.0f;    // rad/s
constexpr float kPitchRateSense = 4.0f;  // rad/s
constexpr float kYawSign = -1.0f;
constexpr float kPitchSign = -1.0f;
constexpr float kYawRateFFSign = 1.0f;
constexpr float kPitchRateFFSign = 1.0f;

// 限位
constexpr float kPitchSoftMinDeg = -35.0f;
constexpr float kPitchSoftMaxDeg = +35.0f;

// 保护
constexpr uint32_t kStartupDelayMs = 1000;
constexpr float kYawRateTrip = 10.0f;
constexpr float kPitchRateTrip = 5.0f;
constexpr uint32_t kRateTripMs = 200;
constexpr float kPitchSafeLimitDeg = 70.0f;

// 调试开关
constexpr bool kImuProbeOnly = false;
constexpr bool kOutputTestZero = false;
constexpr bool kPitchSafeLimitEnable = false;
constexpr bool kYawRateInvert = false;
constexpr bool kPitchRateInvert = false;
constexpr bool kRateLoopInvert = false;

constexpr uint16_t kTxId = 0x1FF;

//==================== 对象 ====================
sp::CAN can1(&hcan1);
sp::PID yaw_rate_pid(kDt, kYawRateKp, kYawRateKi, 0.0f, kYawMaxVoltage, 3.0f);
sp::PID pitch_rate_pid(kDt, kPitchRateKp, kPitchRateKi, 0.0f, kPitchMaxVoltage, kPitchRateMaxIout);
sp::PID yaw_pos_pid(kDt, kYawPosKp, 0.0f, 0.0f, kYawMaxRate, 0.0f);
sp::PID pitch_pos_pid(kDt, kPitchPosKp, 0.0f, 0.0f, kPitchMaxRate, 0.0f);

//==================== 状态 ====================
bool g_started = false;
uint32_t g_start_ms = 0;
bool g_angle_inited = false;
bool g_yaw_angle_ready = false;
float g_yaw_angle_deg_cont = 0.0f;
float g_yaw_angle_raw_last = 0.0f;
float g_q_last[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint32_t g_q_same_count = 0;
float g_yaw_rate_f = 0.0f;
float g_pitch_rate_f = 0.0f;
bool g_filter_ready = false;
bool g_runaway_active = false;
uint32_t g_runaway_since = 0;
uint32_t g_loop_count = 0;
uint32_t g_hz_window_start = 0;

//==================== 工具函数 ====================
float stick_of(float raw)
{
  const float v = sp::limit_min_max(raw, -1.0f, 1.0f);
  return (std::fabs(v) < kStickDeadzone) ? 0.0f : v;
}

// yaw 解卷绕：imu.yaw 只有 (-pi, pi]，这里累加成连续角（度）
float update_yaw_angle_deg(float raw_rad)
{
  if (!g_yaw_angle_ready) {
    g_yaw_angle_deg_cont = raw_rad * 57.29577951f;
    g_yaw_angle_raw_last = raw_rad;
    g_yaw_angle_ready = true;
    return g_yaw_angle_deg_cont;
  }
  g_yaw_angle_deg_cont += sp::limit_angle(raw_rad - g_yaw_angle_raw_last) * 57.29577951f;
  g_yaw_angle_raw_last = raw_rad;
  return g_yaw_angle_deg_cont;
}

// 用共轭四元数把世界竖直轴转到机体系
void world_z_to_body(const float q[4], float out[3])
{
  const float w = q[0], x = -q[1], y = -q[2], z = -q[3];
  const float tx = y * 1.0f + w * 0.0f;
  const float ty = -x * 1.0f + w * 0.0f;
  const float tz = w * 1.0f;
  out[0] = 2.0f * (y * tz - z * ty);
  out[1] = 2.0f * (z * tx - x * tz);
  out[2] = 1.0f + 2.0f * (x * ty - y * tx);
}

void update_imu_stale()
{
  const bool same = (imu.q[0] == g_q_last[0]) && (imu.q[1] == g_q_last[1]) &&
                    (imu.q[2] == g_q_last[2]) && (imu.q[3] == g_q_last[3]);
  if (same) {
    if (g_q_same_count < 1000000U) g_q_same_count++;
  }
  else {
    g_q_same_count = 0;
    for (int i = 0; i < 4; i++) g_q_last[i] = imu.q[i];
  }
  g_imu_stale = (g_q_same_count >= 100U);
}

void update_rate_filter(float yaw_raw, float pitch_raw)
{
  if (!g_filter_ready) {
    g_yaw_rate_f = yaw_raw;
    g_pitch_rate_f = pitch_raw;
    g_filter_ready = true;
    return;
  }
  g_yaw_rate_f = kRateAlpha * g_yaw_rate_f + (1.0f - kRateAlpha) * yaw_raw;
  g_pitch_rate_f = kRateAlpha * g_pitch_rate_f + (1.0f - kRateAlpha) * pitch_raw;
}

// 局部帧头发送，避免 sp::CAN::send 的静态帧头被多任务抢占
void send_frame()
{
  uint8_t frame[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  gimbal_yaw.write(frame);
  gimbal_pitch.write(frame);

  CAN_TxHeaderTypeDef header = {0};
  header.StdId = kTxId;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
    g_tx_drop_count++;
    return;
  }
  uint32_t mailbox = 0;
  HAL_CAN_AddTxMessage(&hcan1, &header, frame, &mailbox);
}

void stop_gimbal()
{
  yaw_rate_pid.clear();
  pitch_rate_pid.clear();
  yaw_pos_pid.clear();
  pitch_pos_pid.clear();
  g_angle_inited = false;
  gimbal_yaw.cmd(0.0f);
  gimbal_pitch.cmd(0.0f);
  gimbal_yaw_out_v = 0.0f;
  gimbal_pitch_out_v = 0.0f;
  send_frame();
}

// 左拨码 → 模式；切换时清 PID、重新对齐目标角，避免带旧状态切换引起跳变
void update_mode(uint32_t now)
{
  uint8_t m = kModeOff;
  if (remote.is_alive(now)) {
    if (remote.sw_l == sp::DBusSwitchMode::UP)
      m = kModeAutoAim;
    else if (remote.sw_l == sp::DBusSwitchMode::MID)
      m = kModeOperator;
    else
      m = kModeOff;
  }

  if (m == g_mode) return;

  g_mode = m;
  g_mode_last = m;
  yaw_rate_pid.clear();
  pitch_rate_pid.clear();
  yaw_pos_pid.clear();
  pitch_pos_pid.clear();
  g_angle_inited = false;
  g_runaway_active = false;
  g_rate_runaway = false;
}

//==================== 主循环 ====================
void gimbal_step()
{
  const uint32_t now = osKernelSysTick();

  g_loop_count++;
  if ((now - g_hz_window_start) >= 500U) {
    g_gimbal_loop_hz = g_loop_count * 2U;
    g_loop_count = 0;
    g_hz_window_start = now;
  }

  if (!g_started) {
    g_started = true;
    g_start_ms = now;
  }
  if ((now - g_start_ms) < kStartupDelayMs) {
    stop_gimbal();
    return;
  }

  update_imu_stale();
  if (g_imu_stale) {
    stop_gimbal();
    return;
  }

  // 关节角速度 = 机体角速度在关节轴上的投影
  float z_body[3];
  world_z_to_body(imu.q, z_body);
  const float yaw_rate_q =
    bmi088.gyro[0] * z_body[0] + bmi088.gyro[1] * z_body[1] + bmi088.gyro[2] * z_body[2];
  const float pitch_rate_q = bmi088.gyro[1];

  const float yaw_raw = sp::limit_min_max(yaw_rate_q, -kRateMeasLimit, kRateMeasLimit);
  const float pitch_raw = sp::limit_min_max(pitch_rate_q, -kRateMeasLimit, kRateMeasLimit);

  gimbal_yaw_rate_raw = yaw_raw;
  gimbal_pitch_rate_raw = pitch_raw;
  gimbal_yaw_rate_euler = imu.vyaw;
  gimbal_pitch_rate_euler = imu.vpitch;
  gimbal_pitch_geom_deg = imu.pitch_geom * 57.29577951f;

  if (kImuProbeOnly) {
    stop_gimbal();
    return;
  }

  // 角度反馈
  const float yaw_deg = update_yaw_angle_deg(imu.yaw);
  const float pitch_deg = imu.pitch * 57.29577951f;
  gimbal_yaw_angle_deg = yaw_deg;
  gimbal_pitch_angle_deg = pitch_deg;

  // 模式：左拨码 上=自瞄(位控) 中=操作手(速控) 下=断电
  update_mode(now);
  if (g_mode == kModeOff) {
    stop_gimbal();
    return;
  }

  // 摇杆
  float yaw_stick = 0.0f;
  float pitch_stick = 0.0f;
  if (remote.is_alive(now)) {
    yaw_stick = stick_of(remote.ch_rh);
    pitch_stick = stick_of(remote.ch_rv);
  }
  gimbal_yaw_stick = yaw_stick;
  gimbal_pitch_stick = pitch_stick;

  if (g_mode == kModeOperator) {
    // 速控：摇杆 × 灵敏度 → 直接给目标角速度，位置环不参与
    gimbal_yaw_rate_cmd = kYawSign * yaw_stick * kYawRateSense;
    gimbal_pitch_rate_cmd = kPitchSign * pitch_stick * kPitchRateSense;
    g_angle_inited = false;  // 切回位控时重新对齐，避免跳变
  }
  else {
    // 位控（自瞄）：目标角积分 + 位置环
    if (!g_angle_inited) {
      gimbal_yaw_target_deg = yaw_deg;
      gimbal_pitch_target_deg = pitch_deg;
      yaw_pos_pid.clear();
      pitch_pos_pid.clear();
      g_angle_inited = true;
    }
    gimbal_yaw_target_deg += kYawSign * yaw_stick * kYawLeverDegPerS * kDt;
    gimbal_pitch_target_deg += kPitchSign * pitch_stick * kPitchLeverDegPerS * kDt;
    gimbal_pitch_target_deg =
      sp::limit_min_max(gimbal_pitch_target_deg, kPitchSoftMinDeg, kPitchSoftMaxDeg);

    // 位置环：误差换成弧度
    constexpr float deg2rad = 0.0174532925f;
    const float yaw_pos_err_deg = gimbal_yaw_target_deg - yaw_deg;
    const float pitch_pos_err_deg = gimbal_pitch_target_deg - pitch_deg;
    gimbal_yaw_pos_err = yaw_pos_err_deg;
    gimbal_pitch_pos_err = pitch_pos_err_deg;

    const float yaw_rad = yaw_deg * deg2rad;
    const float pitch_rad = pitch_deg * deg2rad;
    float yaw_err_rad = yaw_pos_err_deg * deg2rad;
    float pitch_err_rad = pitch_pos_err_deg * deg2rad;
    const float dead_rad = kPosDeadzoneDeg * deg2rad;
    if (std::fabs(yaw_err_rad) < dead_rad) yaw_err_rad = 0.0f;
    if (std::fabs(pitch_err_rad) < dead_rad) pitch_err_rad = 0.0f;

    yaw_pos_pid.calc(yaw_rad + yaw_err_rad, yaw_rad);
    pitch_pos_pid.calc(pitch_rad + pitch_err_rad, pitch_rad);

    gimbal_yaw_rate_cmd = yaw_pos_pid.out;
    gimbal_pitch_rate_cmd = pitch_pos_pid.out;
  }

  // 跑飞保护
  const bool overspeed =
    (std::fabs(g_yaw_rate_f) > kYawRateTrip) || (std::fabs(g_pitch_rate_f) > kPitchRateTrip);
  if (g_runaway_active) {
    if (yaw_stick == 0.0f && pitch_stick == 0.0f) {
      g_runaway_active = false;
      g_rate_runaway = false;
    }
  }
  else if (overspeed) {
    if (g_runaway_since == 0) {
      g_runaway_since = now;
    }
    else if ((now - g_runaway_since) >= kRateTripMs) {
      g_runaway_active = true;
      g_rate_runaway = true;
      g_runaway_since = 0;
      stop_gimbal();
      return;
    }
  }
  else {
    g_runaway_since = 0;
  }
  if (g_runaway_active) {
    stop_gimbal();
    return;
  }

  update_rate_filter(yaw_raw, pitch_raw);
  gimbal_yaw_rate = g_yaw_rate_f;
  gimbal_pitch_rate = g_pitch_rate_f;

  // 速度环
  const float yaw_fb = (kYawRateInvert != kRateLoopInvert) ? -g_yaw_rate_f : g_yaw_rate_f;
  const float pitch_fb = (kPitchRateInvert != kRateLoopInvert) ? -g_pitch_rate_f : g_pitch_rate_f;

  const float yaw_err = gimbal_yaw_rate_cmd - yaw_fb;
  const float pitch_err = gimbal_pitch_rate_cmd - pitch_fb;
  gimbal_yaw_rate_err = yaw_err;
  gimbal_pitch_rate_err = pitch_err;

  const float yaw_err_dz = (std::fabs(yaw_err) < kYawRateDeadzone) ? 0.0f : yaw_err;
  const float pitch_err_dz = (std::fabs(pitch_err) < kPitchRateDeadzone) ? 0.0f : pitch_err;

  yaw_rate_pid.calc(yaw_fb + yaw_err_dz, yaw_fb, kIntegralPause);
  pitch_rate_pid.calc(pitch_fb + pitch_err_dz, pitch_fb, kIntegralPause);

  // 前馈 + P/I
  const float yaw_ff = kYawRateFF * kYawRateFFSign * gimbal_yaw_rate_cmd;
  const float pitch_ff = kPitchRateFF * kPitchRateFFSign * gimbal_pitch_rate_cmd;

  gimbal_yaw_out_v = sp::limit_min_max(yaw_ff + yaw_rate_pid.out, -kYawMaxVoltage, kYawMaxVoltage);
  gimbal_pitch_out_v =
    sp::limit_min_max(pitch_ff + pitch_rate_pid.out, -kPitchMaxVoltage, kPitchMaxVoltage);

  if (kOutputTestZero) {
    gimbal_yaw_out_v = 0.0f;
    gimbal_pitch_out_v = 0.0f;
  }

  // 记录仪
  {
    const bool trigger =
      kOutputTestZero || (std::fabs(pitch_stick) > 0.15f) || (std::fabs(yaw_stick) > 0.15f);
    if (g_rec_reset_req) {
      g_rec_reset_req = 0;
      g_rec_state = 0;
      g_rec_n = 0;
    }
    if (g_rec_state == 0 && trigger) {
      g_rec_state = 1;
      g_rec_n = 0;
      g_rec_tick = 0.0f;
    }
    if (g_rec_state == 1) {
      if (g_rec_n < kRecLen) {
        RecSample & s = rec_buf[g_rec_n];
        s.t_ms = g_rec_tick;
        s.pitch_target_deg = gimbal_pitch_target_deg;
        s.pitch_angle_deg = pitch_deg;
        s.pitch_rate_cmd = gimbal_pitch_rate_cmd;
        s.pitch_rate = g_pitch_rate_f;
        s.pitch_out_v = gimbal_pitch_out_v;
        s.yaw_rate_cmd = gimbal_yaw_rate_cmd;
        s.yaw_rate = g_yaw_rate_f;
        g_rec_n++;
        g_rec_tick += kDt * 1000.0f;
      }
      else if (kOutputTestZero) {
        g_rec_n = 0;
        g_rec_tick = 0.0f;
      }
      else {
        g_rec_state = 2;
      }
    }
  }

  gimbal_yaw.cmd(gimbal_yaw_out_v);
  gimbal_pitch.cmd(gimbal_pitch_out_v);
  send_frame();
}
}  // namespace

extern "C" void gimbal_task(void const * argument)
{
  (void)argument;
  can1.config();
  can1.start();
  for (;;) {
    gimbal_step();
    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;
  const uint32_t stamp_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    can1.recv();
    if (can1.rx_id == gimbal_yaw.rx_id)
      gimbal_yaw.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == gimbal_pitch.rx_id)
      gimbal_pitch.read(can1.rx_data, stamp_ms);
  }
}

// bxCAN 进 bus-off 后发送器被硬件关闭，必须重新初始化才能恢复
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;
  g_can_err_count++;
  const uint32_t esr = hcan->Instance->ESR;
  g_can_tec = (esr & CAN_ESR_TEC) >> 16;
  g_can_rec = (esr & CAN_ESR_REC) >> 24;
  g_can_lec = esr & CAN_ESR_LEC;
  if (esr & CAN_ESR_BOFF) {
    g_can_busoff_count++;
    HAL_CAN_Stop(hcan);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
}
