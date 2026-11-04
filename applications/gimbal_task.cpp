// [解析] 云台控制链路 · 第二步：IMU 角速度内环（开环 → 单速度环）。Yaw = GM6020 ID1，Pitch = ID2，都在 CAN1。
// [解析] 结构（参照狼牙 2022）：摇杆 → 目标角速度 → 速度环（反馈 = C 板 IMU 角速度）→ 电压 → 0x1FF。
// [解析] 内环为什么用 IMU 角速度而不是电机转速：陀螺测的是"云台在世界系的真实转速"，
// [解析]   不受齿隙/联轴器间隙影响，且自带高频阻尼，比编码器差分更容易稳。
// [解析] ★ 参数取最保守的一档（Kp=1.5 V/(rad/s)、限压 6V、目标转速 0.8rad/s）：先求"能动且不抖"。
// [解析] ★ 符号安全：速度环是负反馈，"反馈符号"与"电压→转向"必须一致。若一推杆就朝一个方向加速跑飞，
// [解析]   把 kRateLoopInvert 改成 true。另有"跑飞保护"兜底（kRateLimitTrip）。

#include "cmsis_os.h"

#include <cmath>

#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;   // 定义在 uart_task.cpp（1 kHz 解析 DBus）
extern sp::Mahony imu;    // 定义在 imu_task.cpp（1 kHz 更新姿态与角速度）
extern sp::BMI088 bmi088;  // 定义在 imu_task.cpp（机体角速度，已按 r_ab 转到云台系）

// [解析] GM6020_V = 电压控制：指令帧 0x1FF，ID1→data[0..1]、ID2→data[2..3]，大端 int16。
sp::RM_Motor gimbal_yaw(1, sp::RM_Motors::GM6020_V, 1.0f);
sp::RM_Motor gimbal_pitch(2, sp::RM_Motors::GM6020_V, 1.0f);

// [解析] ===== Live Watch =====
float gimbal_yaw_out_v = 0.0f;     // 本拍发给 Yaw 的电压
float gimbal_pitch_out_v = 0.0f;   // 本拍发给 Pitch 的电压
float gimbal_yaw_stick = 0.0f;     // 右摇杆水平量（已过死区）
float gimbal_pitch_stick = 0.0f;   // 右摇杆垂直量
float gimbal_yaw_rate_cmd = 0.0f;  // 目标角速度 rad/s
float gimbal_pitch_rate_cmd = 0.0f;
float gimbal_yaw_rate = 0.0f;      // 实测角速度（四元数投影 + 低通）rad/s
float gimbal_pitch_rate = 0.0f;
float gimbal_yaw_rate_raw = 0.0f;    // 实测角速度（未滤波，四元数投影）
float gimbal_pitch_rate_raw = 0.0f;
float gimbal_yaw_rate_euler = 0.0f;  // 对照用：旧的欧拉角速率 imu.vyaw（万向锁时会发散，仅诊断）
float gimbal_pitch_rate_euler = 0.0f;
float gimbal_pitch_geom_deg = 0.0f;  // 几何 pitch 角（±180°，低头为正），用来做安全限位
float gimbal_yaw_rate_err = 0.0f;    // 速度环误差（已含极性）
float gimbal_pitch_rate_err = 0.0f;
bool g_pitch_limit_tripped = false;  // pitch 超过安全角，已切断输出

// [解析] ===== 定压探针的极值锁存（Live Watch 刷不过来时用这个看结果）=====
// [解析] 每次 "+V 窗口" 内记录 pitch 转速的最大/最小值，窗口结束后**保持不动**，
// [解析]   所以转完停下来再看也来得及：看 g_probe_max / g_probe_min 的符号就够判断极性。
float g_probe_max = 0.0f;   // 最近一个 +V 窗口内 vpitch 的最大值
float g_probe_min = 0.0f;   // 最近一个 +V 窗口内 vpitch 的最小值
bool g_probe_on = false;    // 当前是否处于 +V 窗口
uint32_t g_gimbal_loop_hz = 0;       // 实测循环频率，正常 ≈1000
uint32_t g_tx_drop_count = 0;        // 邮箱满导致没发出去的次数
bool g_imu_stale = false;            // BMI088 数据连续多拍没变化
bool g_rate_runaway = false;         // 跑飞保护已触发（摇杆回中才恢复）

namespace
{
/*==================== 参数（最保守一档）====================*/
constexpr float kDt = 1e-3f;  // 控制周期，必须与 osDelay(1) 一致

// [解析] 目标角速度上限：1.2 rad/s ≈ 69°/s。
constexpr float kMaxRate = 1.2f;

// [解析] 速度环增益：0.8rad/s 误差 → 1.2V。刻意压低，宁可慢也不要抖。
// [解析] ★ 分轴给：pitch 曾出现"一动就疯狂抖动"，若确诊是环路太硬，单独把 pitch 压低即可。
constexpr float kYawRateKp = 1.5f;    // V / (rad/s)
constexpr float kPitchRateKp = 1.0f;  // V / (rad/s)：pitch 再软一档

// [解析] 积分：默认 0（纯 P 最稳）。如果确诊是"重力+静摩擦"的 stick-slip（卡住→滑落→再卡住），
// [解析]   把它加到 0.5~1.0 让环路能慢慢把扭矩顶上去，就不会卡-滑了。积分限幅很小，风险可控。
constexpr float kYawRateKi = 0.0f;
constexpr float kPitchRateKi = 0.0f;      // ← 重力滑落时改 0.8
constexpr float kPitchRateMaxIout = 3.0f;  // 积分输出限幅 V（只在 Ki>0 时起作用）

// [解析] 电压硬限幅。★ 分轴：pitch 一旦失控最容易撞限位，先给它一个更小的上限（3V）兜住。
constexpr float kYawMaxVoltage = 6.0f;
constexpr float kPitchMaxVoltage = 4.0f;  // 速度保守阶段先压住，确认稳定后再放宽

// [解析] 误差死区：|目标-实测| 小于它就不动作。
// [解析] ★ 这是"轻碰就高频抖"的**第一个旋钮**：死区必须明显大于陀螺噪声（BMI088 静止噪声约 0.02~0.05 rad/s），
// [解析]   否则静止时误差在噪声里反复穿越 0，环路一直点电机 → 高频颤。
constexpr float kRateDeadzone = 0.15f;  // rad/s ≈ 8.6°/s

// [解析] 反馈一阶低通：alpha 越大越平滑、相位滞后越大。
// [解析] ★ 这是第二个旋钮：0.85 ≈ 6.3ms 时间常数，把陀螺噪声压掉一大截（代价是响应变钝一点）。
constexpr float kRateAlpha = 0.85f;

// [解析] 积分分离阈值（单位：伏特，比较 |P 项输出|）。只对 Ki>0 的轴起作用。
constexpr float kIntegralPause = 1.5f;

// [解析] 摇杆死区；上电静默。
constexpr float kStickDeadzone = 0.06f;
constexpr uint32_t kStartupDelayMs = 1000;

// [解析] 方向（2026-11-03 实测校正）：
// [解析]   原 +1/+1 时：左推 → yaw 顺时针、上推 → 俯下（两个都反）
// [解析]   现在 -1/-1：右推 → yaw 顺时针、上推 → 抬头
// [解析] ★ 这两个符号只影响"摇杆方向"，不影响环路极性；环路极性由 kYawRateInvert/kPitchRateInvert 管。
constexpr float kYawSign = -1.0f;
constexpr float kPitchSign = -1.0f;

// [解析] ★ 分轴负反馈极性开关：某轴"推杆后越转越快/撞限位抖"就是这个轴反了，单独翻它。
// [解析]   两轴的正转方向与 C 板 +x/+y 轴是相互独立的，所以必须分开判断（同一条总线上也常见一轴对一轴反）。
constexpr bool kYawRateInvert = false;
// [解析] 2026-11-03 第一轮：pitch 抖动 → 翻 true 后变成"一动就转 180°"。
// [解析]   说明不是极性，而是**反馈根本没读到 pitch 转速**（开环跑飞）。等轴映射查清后再定这个值。
constexpr bool kPitchRateInvert = false;

// [解析] ★★ IMU 轴映射探测模式：两轴恒输出 0V、不进速度环，只把 IMU 的角速度实时显示出来。
// [解析]   2026-11-03 已用它确认：绕 pitch 轴转只有 imu.vpitch 变（俯头正、仰头负）→ 轴映射正确、r_ab 用单位阵。
constexpr bool kImuProbeOnly = false;

// [解析] ★★ 极性探针（当前启用）：>0 时进入"定压测试" —— 不闭环，pitch 每 1.5s 交替给 +kProbeVoltage / 0V。
// [解析]   用法：看 gimbal_pitch_rate_raw 的符号，就能直接判定速度环该怎么接：
// [解析]     V>0 时 vpitch 变**正**  → 电压正 = 俯头 → 反馈取正，kPitchRateInvert = false
// [解析]     V>0 时 vpitch 变**负**  → 电压正 = 仰头 → 反馈要取反，kPitchRateInvert = true
// [解析]   这个实验不闭环、电压只有 1.5V，绝对不会跑飞；测完改回 0。
constexpr float kProbeVoltage = 0.0f;  // 0 = 正常闭环（当前）；>0 = 定压极性探针
constexpr uint32_t kProbeOnMs = 2500;  // 探针每段 2.5s

// [解析] 保留旧名字做兼容（全局翻转会对两轴同时生效，排查时用）。
constexpr bool kRateLoopInvert = false;

// [解析] 跑飞保护：实测角速度超过这个值并持续 200ms → 断电锁定，摇杆回中恢复。
// [解析] ★ 阈值必须显著大于 kMaxRate，否则正常快速转动会被误判成跑飞。
constexpr float kRateLimitTrip = 4.0f;  // rad/s ≈ 229°/s（kMaxRate=1.2 的 3.3 倍）
constexpr uint32_t kRateTripMs = 200;

// [解析] 陀螺投影没有 1/cos 发散问题，但保留一个物理上限防止异常值。
constexpr float kRateMeasLimit = 6.0f;  // rad/s

// [解析] pitch 安全限位开关（当前**关闭**，用于观察万向锁附近的原始现象）。
// [解析]   为什么关：切断输出=电机自由，云台会翻过垂直后永久无法解除锁定（详见 gimbal_step 里的注释）。
constexpr bool kPitchSafeLimitEnable = false;

// [解析] pitch 安全限位：几何 pitch 角超过它就直接切断输出。
// [解析]   用 pitch_geom（±180°）而不是 pitch（asin，只有 ±90°），否则过了垂直之后 asin 会"折回来"看不出超限。
constexpr float kPitchSafeLimitDeg = 70.0f;

constexpr uint16_t kTxId = 0x1FF;

/*==================== 对象 ====================*/
// [解析] 速度环：默认纯 P（Ki=0）。pitch 若受重力影响严重，可把 kPitchRateKi 打开做积分。
// [解析] max_iout 只在 Ki>0 时起作用，用来防止积分把电压顶到限幅。
sp::PID yaw_rate_pid(kDt, kYawRateKp, kYawRateKi, 0.0f, kYawMaxVoltage, 3.0f);
sp::PID pitch_rate_pid(kDt, kPitchRateKp, kPitchRateKi, 0.0f, kPitchMaxVoltage,
                       kPitchRateMaxIout);

/*==================== 状态 ====================*/
uint32_t g_loop_count = 0;
uint32_t g_hz_window_start = 0;
bool g_started = false;
uint32_t g_start_ms = 0;

// IMU 是否在更新：四元数连续多拍不变就判定没数据
float g_q_last[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint32_t g_q_same_count = 0;

// 反馈低通状态
float g_yaw_rate_f = 0.0f;
float g_pitch_rate_f = 0.0f;
bool g_filter_ready = false;

// 跑飞保护状态
bool g_runaway_active = false;
uint32_t g_runaway_since = 0;

/*==================== 内部函数 ====================*/
float stick_of(float raw)
{
  const float v = sp::limit_min_max(raw, -1.0f, 1.0f);
  return (std::fabs(v) < kStickDeadzone) ? 0.0f : v;
}

// [解析] BMI088 是否在更新（bmi088.init() 找不到芯片会卡在重试里，那时 q 永远是初值）。
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

// [解析] 一阶低通：alpha 越大越平滑。1kHz 下 alpha=0.7 的时间常数约 3ms。
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

// [解析] 用四元数把**世界竖直轴**转到机体系：v_body = q* ⊗ v_world ⊗ q（q 是机体→世界的姿态四元数，wxyz）。
// [解析] 只做这一件事，所以内联实现 —— sp::Mahony 里那个 quaternion_frame_transform_for_mahony 是 private。
// [解析] 与中间件里同一套公式：v_out = v_in + 2·cross(q_vec, cross(q_vec, v_in) + w·v_in)。
void world_z_to_body(const float q[4], float out[3])
{
  const float w = q[0];
  const float x = -q[1];  // 取共轭：world → body
  const float y = -q[2];
  const float z = -q[3];

  const float vx = 0.0f, vy = 0.0f, vz = 1.0f;  // 世界竖直轴

  const float tx = y * vz - z * vy + w * vx;
  const float ty = z * vx - x * vz + w * vy;
  const float tz = x * vy - y * vx + w * vz;

  out[0] = vx + 2.0f * (y * tz - z * ty);
  out[1] = vy + 2.0f * (z * tx - x * tz);
  out[2] = vz + 2.0f * (x * ty - y * tx);
}

// [解析] 把两台电机的 2 字节指令拼进一帧 0x1FF 并发出。
// [解析] 不用 sp::CAN::send()：它的帧头是函数级 static，和 chassis_task 的发送共用会被抢占写坏。
void send_frame()
{
  uint8_t frame[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  gimbal_yaw.write(frame);    // ID1 → frame[0..1]
  gimbal_pitch.write(frame);  // ID2 → frame[2..3]

  CAN_TxHeaderTypeDef header = {0};
  header.StdId = kTxId;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
    g_tx_drop_count++;  // 邮箱满：丢掉本帧，下一拍再来（不在任务里忙等）
    return;
  }
  uint32_t mailbox = 0;
  HAL_CAN_AddTxMessage(&hcan1, &header, frame, &mailbox);
}

// [解析] 统一出口：清 PID、下发 0V。任何异常都走这里。
void stop_gimbal()
{
  yaw_rate_pid.clear();
  pitch_rate_pid.clear();
  gimbal_yaw.cmd(0.0f);
  gimbal_pitch.cmd(0.0f);
  gimbal_yaw_out_v = 0.0f;
  gimbal_pitch_out_v = 0.0f;
  send_frame();
}

/*==================== 主循环体 ====================*/
void gimbal_step()
{
  const uint32_t now = osKernelSysTick();

  // ---- 诊断：实测循环频率（500ms 窗口）----
  g_loop_count++;
  if ((now - g_hz_window_start) >= 500U) {
    g_gimbal_loop_hz = g_loop_count * 2U;
    g_loop_count = 0;
    g_hz_window_start = now;
  }

  // ---- 上电静默 ----
  if (!g_started) {
    g_started = true;
    g_start_ms = now;
  }
  if ((now - g_start_ms) < kStartupDelayMs) {
    stop_gimbal();
    return;
  }

  // ---- IMU 健康检查：没有角速度就没法做速度环，一律断电 ----
  update_imu_stale();
  if (g_imu_stale) {
    stop_gimbal();
    return;
  }

  // ---- 反馈：四元数投影求"关节角速度"（彻底避开欧拉角的万向节死锁）----
  // [解析] ★ 为什么不能直接用 imu.vyaw / imu.vpitch：
  // [解析]   vyaw = wy·sin(roll)/cos(pitch) + wz·cos(roll)/cos(pitch)，pitch→±90° 时 cos(pitch)→0 → 发散。
  // [解析]   实测现象：pitch 转到垂直，yaw 电机突然猛扭、随后 yaw 失控，而 pitch 还能控 —— 正是这个式的 1/cos。
  // [解析] ★ 正确做法：关节角速度 = 机体角速度在"该关节转轴"上的投影，任何姿态都有定义：
  // [解析]   - yaw 关节轴 = 世界竖直轴 z；把它用四元数转到机体系得到 z_body，再与陀螺点乘；
  // [解析]   - pitch 关节轴 = 机体系 y 轴（绕 y 转不会改变 y 自身方向），直接取 gyro[1]。
  // [解析]   姿态水平时两者分别退化为 gyro[2]、gyro[1]，与旧的 vyaw/vpitch 同号 → 极性和增益都不用改。
  float z_in_body[3] = {0.0f, 0.0f, 0.0f};
  world_z_to_body(imu.q, z_in_body);

  const float yaw_rate_q =
    bmi088.gyro[0] * z_in_body[0] + bmi088.gyro[1] * z_in_body[1] + bmi088.gyro[2] * z_in_body[2];
  const float pitch_rate_q = bmi088.gyro[1];

  const float yaw_raw = sp::limit_min_max(yaw_rate_q, -kRateMeasLimit, kRateMeasLimit);
  const float pitch_raw = sp::limit_min_max(pitch_rate_q, -kRateMeasLimit, kRateMeasLimit);

  gimbal_yaw_rate_raw = yaw_raw;
  gimbal_pitch_rate_raw = pitch_raw;
  gimbal_yaw_rate_euler = imu.vyaw;    // 对照：万向锁附近它会炸，仅用于诊断
  gimbal_pitch_rate_euler = imu.vpitch;
  gimbal_pitch_geom_deg = imu.pitch_geom * 57.29577951f;

  // ---- 安全限位（可关）：pitch 接近垂直就切断 ----
  // [解析] ★ 2026-11-04 实测发现这个策略有缺陷：切断=0V=电机自由，云台会在重力下翻过垂直，
  // [解析]   之后 pitch_geom 回不到解除阈值 → **永久锁定，正反转都不响应**。
  // [解析]   所以先整体关掉（kPitchSafeLimitEnable = false），用来观察万向锁附近的原始现象。
  // [解析]   将来要恢复，应该改成"只在朝超限方向运动时限幅"，不要切断全部扭矩。
  if (kPitchSafeLimitEnable) {
    const bool pitch_over = std::fabs(gimbal_pitch_geom_deg) > kPitchSafeLimitDeg;
    if (pitch_over) {
      g_pitch_limit_tripped = true;
    }
    else if (std::fabs(gimbal_pitch_geom_deg) < (kPitchSafeLimitDeg - 5.0f)) {
      g_pitch_limit_tripped = false;  // 5° 回差，避免在边界反复切
    }
    if (g_pitch_limit_tripped) {
      gimbal_yaw_rate_cmd = 0.0f;
      gimbal_pitch_rate_cmd = 0.0f;
      gimbal_yaw_stick = 0.0f;
      gimbal_pitch_stick = 0.0f;
      stop_gimbal();
      return;
    }
  }
  else {
    g_pitch_limit_tripped = false;
  }

  // [解析] 探测模式：两轴 0V，不进环路 —— 用手转云台，看三个角速度哪个跟着动。
  if (kImuProbeOnly) {
    gimbal_yaw_rate_cmd = 0.0f;
    gimbal_pitch_rate_cmd = 0.0f;
    gimbal_yaw_stick = 0.0f;
    gimbal_pitch_stick = 0.0f;
    stop_gimbal();
    return;
  }

  // [解析] 极性探针：定压、不闭环。pitch 每隔 kProbeOnMs 交替在 +kProbeVoltage 和 0 之间切换，
  // [解析]   这样看 gimbal_pitch_rate_raw 的符号就能判定电压与转速的对应关系。
  if (kProbeVoltage > 0.0f) {
    const bool on = ((now / kProbeOnMs) % 2U) == 0U;
    const float v = on ? kProbeVoltage : 0.0f;

    // [解析] 窗口开始时清空极值，窗口内持续更新；窗口结束后保持不变（锁存），方便事后查看。
    if (on && !g_probe_on) {
      g_probe_max = pitch_raw;
      g_probe_min = pitch_raw;
    }
    else if (on) {
      if (pitch_raw > g_probe_max) g_probe_max = pitch_raw;
      if (pitch_raw < g_probe_min) g_probe_min = pitch_raw;
    }
    g_probe_on = on;

    gimbal_yaw.cmd(0.0f);
    gimbal_pitch.cmd(v);
    gimbal_yaw_out_v = 0.0f;
    gimbal_pitch_out_v = v;
    gimbal_yaw_rate_cmd = 0.0f;
    gimbal_pitch_rate_cmd = 0.0f;
    gimbal_yaw_stick = 0.0f;
    gimbal_pitch_stick = 0.0f;
    send_frame();
    return;
  }

  update_rate_filter(yaw_raw, pitch_raw);
  gimbal_yaw_rate = g_yaw_rate_f;
  gimbal_pitch_rate = g_pitch_rate_f;

  // ---- 摇杆 → 目标角速度 ----
  float yaw_stick = 0.0f;
  float pitch_stick = 0.0f;
  if (remote.is_alive(now)) {  // 遥控器失联 → 目标 0，速度环主动把云台刹住
    yaw_stick = stick_of(remote.ch_rh);
    pitch_stick = stick_of(remote.ch_rv);
  }
  gimbal_yaw_stick = yaw_stick;
  gimbal_pitch_stick = pitch_stick;

  gimbal_yaw_rate_cmd = kYawSign * yaw_stick * kMaxRate;
  gimbal_pitch_rate_cmd = kPitchSign * pitch_stick * kMaxRate;

  // ---- 跑飞保护：实测转速远超指令上限 → 断定正反馈/失控 ----
  const bool overspeed = (std::fabs(g_yaw_rate_f) > kRateLimitTrip) ||
                         (std::fabs(g_pitch_rate_f) > kRateLimitTrip);
  if (g_runaway_active) {
    if (yaw_stick == 0.0f && pitch_stick == 0.0f) {
      g_runaway_active = false;  // 摇杆回中：解除
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

  // ---- 速度环：目标角速度 − 实测角速度（分轴极性 + 死区）→ 电压 ----
  // [解析] 极性翻转作用在"反馈符号"上：某轴反了就把它取负，把正反馈变回负反馈。
  const float yaw_fb = (kYawRateInvert != kRateLoopInvert) ? -g_yaw_rate_f : g_yaw_rate_f;
  const float pitch_fb = (kPitchRateInvert != kRateLoopInvert) ? -g_pitch_rate_f : g_pitch_rate_f;

  const float yaw_err = gimbal_yaw_rate_cmd - yaw_fb;
  const float pitch_err = gimbal_pitch_rate_cmd - pitch_fb;
  gimbal_yaw_rate_err = yaw_err;
  gimbal_pitch_rate_err = pitch_err;

  // [解析] 死区在这里实现（sp::PID 没有死区参数）：误差小于阈值时把它变成 0。
  const float yaw_err_dz = (std::fabs(yaw_err) < kRateDeadzone) ? 0.0f : yaw_err;
  const float pitch_err_dz = (std::fabs(pitch_err) < kRateDeadzone) ? 0.0f : pitch_err;

  // [解析] 用带"积分分离"的重载：|P 项输出| 大于 kIntegralPause 时不积分，防止大误差时积分顶满。
  // [解析]   sp::PID 内部算 set-fdb，所以把 set 写成 fdb+err 来实现"误差已被处理过"的效果。
  yaw_rate_pid.calc(yaw_fb + yaw_err_dz, yaw_fb, kIntegralPause);
  pitch_rate_pid.calc(pitch_fb + pitch_err_dz, pitch_fb, kIntegralPause);

  gimbal_yaw_out_v = yaw_rate_pid.out;
  gimbal_pitch_out_v = pitch_rate_pid.out;

  gimbal_yaw.cmd(gimbal_yaw_out_v);
  gimbal_pitch.cmd(gimbal_pitch_out_v);
  send_frame();
}

}  // namespace

// [解析] 任务本体：1 kHz 固定节拍，函数不会返回（FreeRTOS 不允许 return）。
extern "C" void gimbal_task(void const * argument)
{
  (void)argument;

  for (;;) {
    gimbal_step();
    osDelay(1);
  }
}

/*==============================================================================
 * 使用与验证
 *
 * 上电：静默 1s → 之后推杆云台按"摇杆量 × 0.8rad/s"的速度转，松手即刹停。
 * 注意：这是**速度环**，不是位置环 —— 松手后云台停在哪儿就是哪儿，不会回到某个角度。
 *
 * | 变量                  | 正常值 / 含义                                  |
 * |-----------------------|------------------------------------------------|
 * | g_gimbal_loop_hz      | ≈1000                                          |
 * | gimbal_yaw_stick      | 中位 0.000                                     |
 * | gimbal_yaw_rate_cmd   | 推满 ±0.8 rad/s                                |
 * | gimbal_yaw_rate       | 跟随 cmd（滤波后）                             |
 * | gimbal_yaw_rate_raw   | 未滤波值；与 rate 差很多说明陀螺噪声大          |
 * | gimbal_yaw_out_v      | 通常在 ±2V 内；贴 ±6V 说明跟不动或符号反        |
 * | g_rate_runaway        | 必须 false                                     |
 * | g_tx_drop_count       | 基本不动                                       |
 *
 * 故障对照：
 * 1) **推杆后越转越快、停不下来** → 正反馈，把 kRateLoopInvert 改成 true（或翻转 kYawSign/kPitchSign 之一）。
 *    跑飞保护会在 200ms 后切断，此时 g_rate_runaway = true，摇杆回中可恢复。
 * 2) **推杆几乎不动，out_v 很小** → Kp 太保守，把 kRateKp 1.5 → 3.0；或电机没收到指令（查 CAN/ID/模式）。
 * 3) **out_v 贴 ±6V（pitch 是 ±3V）但转不起来** → 电压不够顶静摩擦，把上限调大。
 * 4) **嗡嗡抖/高频颤** → 陀螺噪声被放大：把 kRateKp 降到 1.0，或把 kRateAlpha 提到 0.85（更平滑）。
 * 5) **只在一个方向抖** → 该轴陀螺符号与另一轴不一致，检查 kRateLoopInvert 是否只该对单轴生效。
 *
 * 下一步（确认这一版稳了再做）：在前面加位置环（IMU 角度）→ 变成串级，才是完整云台。
 *============================================================================*/
