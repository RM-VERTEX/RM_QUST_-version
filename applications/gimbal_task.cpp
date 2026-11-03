// [解析] 云台任务（验证模型）：Yaw = GM6020 ID1(0x205)，Pitch = GM6020 ID2(0x206)，都在 CAN1。
// [解析] ★ C 板现在固定在云台上，所以外层角度环的反馈改用 C 板 BMI088 的姿态角（imu.yaw / imu.pitch），
// [解析]   这就是 MAS 云台的做法：IMU 直接测云台在世界系的姿态，不依赖机械零位，代码里也没有"零点"概念。
// [解析] 控制结构（照 MAS）：右摇杆当"速度输入"积分成目标角 → 角度环（IMU 姿态）→ 速度环（转子 rad/s）→ 电压。
// [解析] 与 MAS 的两处差异：
// [解析]   1) MAS 的速度环反馈是 6020 的 rpm；这里用 sp::RM_Motor 已换算好的 rad/s，等价；
// [解析]   2) MAS 的 Pitch 用 IMU 的 roll（因为他们的 IMU 侧装）；本车 C 板正装，用 pitch，
// [解析]      若实测发现 pitch 轴转动体现在 roll 上，把 kPitchFromRoll 改成 true 即可。

#include "cmsis_os.h"

#include <cmath>

#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;  // 定义在 uart_task.cpp
extern sp::Mahony imu;   // 定义在 imu_task.cpp（1 kHz 更新）

// [解析] GM6020_V = 电压控制（指令帧 0x1FF，与 MAS 一致）；ratio=1.0（直驱）；multi_circle 让编码器角能连续累计。
sp::RM_Motor gimbal_yaw(1, sp::RM_Motors::GM6020_V, 1.0f);
sp::RM_Motor gimbal_pitch(2, sp::RM_Motors::GM6020_V, 1.0f);

// [解析] Live Watch 用的一批全局量（都不参与控制逻辑，只给人看）。
float gimbal_yaw_target_rad = 0.0f;    // 期望 yaw 角（世界系，连续）
float gimbal_pitch_target_rad = 0.0f;  // 期望 pitch 角（世界系，低头为正）
float gimbal_yaw_out_v = 0.0f;         // 最终下发的电压，单位 V
float gimbal_pitch_out_v = 0.0f;
float gimbal_yaw_stick = 0.0f;         // 右摇杆水平量（已过死区），中位必须是 0
float gimbal_pitch_stick = 0.0f;       // 右摇杆垂直量（已过死区），中位必须是 0
float gimbal_imu_yaw = 0.0f;           // imu.yaw 原始值（会回绕，仅诊断用）
float gimbal_imu_pitch = 0.0f;         // imu.pitch 原始值
bool g_input_latched = false;          // 顶死锁定：为 true 时不接受摇杆，两杆回中才解锁

// [解析] ===== 诊断量（Live Watch 用，不参与控制）=====
uint32_t g_gimbal_loop_hz = 0;  // 实测控制频率，正常应接近 1000；明显偏低说明节拍被拖慢（增益会等效放大）
uint32_t g_yaw_fb_lost = 0;     // Yaw 电机反馈"在线→掉线"次数（CAN 丢帧会涨）
uint32_t g_pitch_fb_lost = 0;   // Pitch 同上
bool g_imu_stale = false;       // true = BMI088 数据连续多拍没变化（没在更新）

namespace
{
/*==================== 帧与量纲 ====================*/
// [解析] GM6020 电压帧 0x1FF 装 ID1~4；本模型只占 ID1/ID2。
constexpr uint16_t kTxId = 0x1FF;

/*==================== 控制参数 ====================*/
constexpr float kDt = 1e-3f;  // 控制周期, 必须与 osDelay(1) 一致

// [解析] 反馈来源：true = C 板 IMU 姿态（C 板装在云台上时用这个）；false = 6020 编码器角（C 板不在云台上时用）。
constexpr bool kFeedbackImu = true;

// [解析] pitch 轴用哪个欧拉角：C 板正装用 imu.pitch；若 C 板侧装/竖装导致 pitch 转动体现在 roll 上，改 true。
constexpr bool kPitchFromRoll = false;

// [解析] 摇杆满偏时"目标角"的变化速率（操作手能给出的最大转动速度）。
// [解析] MAS 用 Gimbal_LeverSpeedMapRate 换算成约 154°/s(pitch) / 346°/s(yaw)，这里取保守值。
constexpr float kYawLeverRate = 4.0f;    // rad/s ≈ 229°/s
constexpr float kPitchLeverRate = 2.0f;  // rad/s ≈ 115°/s
constexpr float kStickDeadzone = 0.06f;

// [解析] 方向：右推 yaw 顺时针（IMU 的 yaw 逆时针为正，故取负）。
// [解析] ★ imu.pitch 是"低头为正"（源码注释 + pitch=asin(2q0q2-2q1q3) 都说明这点），
// [解析]   所以"上推 = 抬头"必须取负。
// [解析] ★ 实机若方向相反，只改这两个符号，不要改 PID 或摇杆逻辑。
constexpr float kYawSign = -1.0f;
constexpr float kPitchSign = -1.0f;

// [解析] Pitch 机械限位（**世界系**，单位 rad，低头为正）：
// [解析]   kPitchMin 是"抬头极限"（负得更多），kPitchMax 是"低头极限"。
// [解析]   现在仍是占位值（±51°），必须按验证模型的实际行程实测后修改。
constexpr float kPitchMin = -0.9f;  // 抬头到 -51°
constexpr float kPitchMax = +0.9f;  // 低头到 +51°

// [解析] 电压上限：GM6020 满量程 ±24V（手册：电压给定 ±25000 ↔ ±24V）。先留 20V 余量。
constexpr float kMaxVoltage = 20.0f;

// [解析] 串级 PID：角度环输出"目标角速度(rad/s)"，速度环输出"电压(V)"。
// [解析] ★ 换 IMU 反馈后外环多出约 1~2ms 采样滞后（imu_task 是 BelowNormal，本任务读到的常是上一拍数据），
// [解析]   而 GM6020 的机械时间常数只有 3ms（手册），所以外环增益必须比对编码器时更低。
// [解析]   这两组是"先求不抖"的保守起步值：确认稳定后再按文件末尾的顺序往上加。
constexpr float kYawPosKp = 3.0f;  // (rad/s) / rad
constexpr float kYawPosKi = 0.0f;  // 起步阶段一律不加积分
constexpr float kYawPosKd = 0.1f;
constexpr float kYawSpdKp = 2.0f;  // V / (rad/s)

constexpr float kPitchPosKp = 3.0f;
constexpr float kPitchPosKi = 0.0f;
constexpr float kPitchPosKd = 0.1f;
constexpr float kPitchSpdKp = 2.0f;

constexpr float kMaxSpeedCmd = 6.0f;   // 角度环输出限幅 rad/s（≈344°/s）
constexpr float kPitchMaxIout = 1.5f;  // pitch 角度环积分限幅 rad/s（加 Ki 后才有意义）
constexpr float kPosAlpha = 0.5f;      // 角度环 D 项一阶滤波（压 IMU / 编码器噪声）

// [解析] 积分分离：只有 |P 项输出| 小于这个值（误差已经不大）才允许积分，防 windup。
constexpr float kIntegralPause = 0.6f;

// [解析] 饱和保护：输出贴限幅连续超过这个时间，判定"顶住/震荡"，清积分 + 退回当前角 + 锁定输入。
constexpr float kSatTripMs = 400;

// [解析] 单轴调试开关：关掉的轴恒输出 0V，且不参与"反馈是否在线"的仲裁。
constexpr bool kEnableYaw = true;
constexpr bool kEnablePitch = true;

// [解析] 二分定位开关：true = 两轴恒发 0V（电机自由），确认抖动是不是闭环引起的。测试时手扶云台。
constexpr bool kZeroVoltageTest = false;

/*==================== 对象 ====================*/
// [解析] 本板只跑云台，CAN1 由本任务独占。
sp::CAN can1(&hcan1);

// [解析] sp::PID 参数顺序: (dt, kp, ki, kd, max_out, max_iout, alpha)
// [解析] 速度环不积分：静差靠角度环消，速度环只做"够快"的内环。
sp::PID yaw_pos_pid(kDt, kYawPosKp, kYawPosKi, kYawPosKd, kMaxSpeedCmd, 0.0f, kPosAlpha);
sp::PID yaw_spd_pid(kDt, kYawSpdKp, 0.0f, 0.0f, kMaxVoltage, 0.0f);
sp::PID pitch_pos_pid(kDt, kPitchPosKp, kPitchPosKi, kPitchPosKd, kMaxSpeedCmd, kPitchMaxIout,
                      kPosAlpha);
sp::PID pitch_spd_pid(kDt, kPitchSpdKp, 0.0f, 0.0f, kMaxVoltage, 0.0f);

/*==================== 状态 ====================*/
bool g_inited = false;  // 是否已用当前姿态初始化过目标

bool g_sat_active = false;
uint32_t g_sat_since_ms = 0;
uint32_t g_sat_trips = 0;

// [解析] yaw 解卷绕状态：imu.yaw 每帧只在 (-π, π] 内，这里累加成连续值。
bool g_yaw_cont_ready = false;
float g_yaw_cont = 0.0f;      // 连续 yaw（世界系，可超过 ±π）
float g_yaw_raw_last = 0.0f;  // 上一拍的 imu.yaw 原始值

// [解析] IMU 是否在更新：四元数连续多拍一模一样就判定为"没在更新"。
float g_q_last[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint32_t g_q_same_count = 0;

// [解析] 诊断：控制频率统计窗口与反馈掉线计数。
uint32_t g_loop_count = 0;
uint32_t g_hz_window_start = 0;
bool g_yaw_was_ok = false;
bool g_pitch_was_ok = false;

/*==================== 内部函数 ====================*/
float deadzone(float v) { return (std::fabs(v) < kStickDeadzone) ? 0.0f : v; }

// [解析] imu.yaw 是 atan2 出来的，转过 ±180° 会突跳 2π。这里用"本帧与上帧之差"折算到 (-π, π] 再累加，
// [解析]   得到连续的 yaw。必须做，否则 yaw 过 ±π 时会瞬间产生 2π 的误差 → 电机猛甩。
float imu_yaw_continuous(float raw)
{
  if (!g_yaw_cont_ready) {
    g_yaw_cont = raw;
    g_yaw_raw_last = raw;
    g_yaw_cont_ready = true;
    return g_yaw_cont;
  }
  g_yaw_cont += sp::limit_angle(raw - g_yaw_raw_last);
  g_yaw_raw_last = raw;
  return g_yaw_cont;
}

// [解析] 取外层角度环的反馈：IMU 模式读姿态角，编码器模式读电机角。
void read_feedback(float & yaw_fdb, float & pitch_fdb)
{
  if (kFeedbackImu) {
    yaw_fdb = imu_yaw_continuous(imu.yaw);
    pitch_fdb = kPitchFromRoll ? imu.roll : imu.pitch;
  }
  else {
    yaw_fdb = gimbal_yaw.angle;
    pitch_fdb = gimbal_pitch.angle;
  }
}

// [解析] BMI088 是否在更新：四元数连续 100 拍不变就当它没数据
// [解析]   （bmi088.init() 找不到芯片时会卡在重试里，那时 q 永远是初值）。
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

// [解析] 一帧 0x1FF 装 2 台 6020 的电压：各写自己那 2 字节（ID1→[0..1]、ID2→[2..3]），再发一次。
void send_voltage()
{
  gimbal_yaw.write(can1.tx_data);
  gimbal_pitch.write(can1.tx_data);
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) can1.send(kTxId);
}

// [解析] 失联/异常的统一出口：清 PID（否则积分残留会在恢复瞬间猛冲）并下发 0V。
void stop_gimbal()
{
  yaw_pos_pid.clear();
  yaw_spd_pid.clear();
  pitch_pos_pid.clear();
  pitch_spd_pid.clear();
  gimbal_yaw.cmd(0.0f);
  gimbal_pitch.cmd(0.0f);
  gimbal_yaw_out_v = 0.0f;
  gimbal_pitch_out_v = 0.0f;
  send_voltage();
}

// [解析] 上电第一次拿到有效反馈时，把目标角对齐当前姿态，避免"一上电就往某个角冲"。
// [解析] Pitch 不做限幅：若初始姿态本来在行程外，限幅会让目标瞬间跳到边界、电机立刻动。
void init_targets()
{
  float y = 0.0f, p = 0.0f;
  read_feedback(y, p);
  gimbal_yaw_target_rad = y;
  gimbal_pitch_target_rad = p;
  g_inited = true;
}

// [解析] 一个控制周期：诊断 → 仲裁 → 摇杆积分 → 串级 PID → 饱和保护 → 下发。
void gimbal_step()
{
  const uint32_t now = osKernelSysTick();

  // ---- 0. 诊断：实测控制频率（500ms 窗口）----
  // [解析] 频率明显低于 1000Hz 说明 osDelay(1) 被拖长；而 PID 的 dt 写死 1e-3，
  // [解析]   等效于把增益放大同样倍数 → 会抖。
  g_loop_count++;
  if ((now - g_hz_window_start) >= 500U) {
    g_gimbal_loop_hz = g_loop_count * 2U;
    g_loop_count = 0;
    g_hz_window_start = now;
  }

  update_imu_stale();
  gimbal_imu_yaw = imu.yaw;
  gimbal_imu_pitch = imu.pitch;

  // ---- 1. 反馈仲裁：在用的信号源失效就全部归零（看不见就不动）----
  const bool yaw_ok = (!kEnableYaw) || gimbal_yaw.is_alive(now);
  const bool pitch_ok = (!kEnablePitch) || gimbal_pitch.is_alive(now);
  const bool imu_ok = (!kFeedbackImu) || !g_imu_stale;

  // [解析] 诊断：统计"在线→掉线"次数。一直涨说明 CAN 反馈断续，
  // [解析]   那样 stop_gimbal() 会周期性把电压清 0，云台表现为抽动（而不是 PID 振荡）。
  if (g_yaw_was_ok && !yaw_ok) g_yaw_fb_lost++;
  if (g_pitch_was_ok && !pitch_ok) g_pitch_fb_lost++;
  g_yaw_was_ok = yaw_ok;
  g_pitch_was_ok = pitch_ok;

  if (!yaw_ok || !pitch_ok || !imu_ok) {
    g_inited = false;  // 恢复后重新对齐目标
    stop_gimbal();
    return;
  }

  // ---- 2. 取反馈（IMU 姿态 或 编码器角）----
  float yaw_fdb = 0.0f;
  float pitch_fdb = 0.0f;
  read_feedback(yaw_fdb, pitch_fdb);
  if (!g_inited) init_targets();

  // ---- 3. 摇杆 → 目标角（摇杆是"速度"输入，积分成角度）----
  // [解析] 用**右摇杆**（ch_rh 水平 / ch_rv 垂直）：DJI 遥控器各手模式下右摇杆都是弹簧回中的。
  // [解析]   不要用 ch_lv：它是"左垂直"，美国手下是油门、不回中，会让目标角一直朝一个方向积分。
  float yaw_stick = 0.0f;
  float pitch_stick = 0.0f;
  if (remote.is_alive(now)) {
    yaw_stick = deadzone(sp::limit_min_max(remote.ch_rh, -1.0f, 1.0f));
    pitch_stick = deadzone(sp::limit_min_max(remote.ch_rv, -1.0f, 1.0f));
  }
  gimbal_yaw_stick = yaw_stick;
  gimbal_pitch_stick = pitch_stick;

  if (g_input_latched) {
    // [解析] 顶死锁定期间：不接受摇杆，目标退回当前姿态（停止死顶）；两杆都回中才解锁。
    gimbal_yaw_target_rad = yaw_fdb;
    gimbal_pitch_target_rad = pitch_fdb;
    if (std::fabs(yaw_stick) < kStickDeadzone && std::fabs(pitch_stick) < kStickDeadzone) {
      g_input_latched = false;
    }
  }
  else {
    gimbal_yaw_target_rad += kYawSign * yaw_stick * kYawLeverRate * kDt;
    gimbal_pitch_target_rad += kPitchSign * pitch_stick * kPitchLeverRate * kDt;
    gimbal_pitch_target_rad = sp::limit_min_max(gimbal_pitch_target_rad, kPitchMin, kPitchMax);
  }
  // 遥控器失联时保持目标角不变：电机仍闭环撑住姿态，不会自己掉下来

  // ---- 4. 串级 PID：角度环（IMU 姿态）→ 速度环（转子 rad/s）→ 电压 ----
  // [解析] 角度环用带"积分分离"的重载：|P 项| 大于阈值时不积分，被限位/被顶住时不会攒积分。
  yaw_pos_pid.calc(gimbal_yaw_target_rad, yaw_fdb, kIntegralPause);
  yaw_spd_pid.calc(yaw_pos_pid.out, gimbal_yaw.speed);
  pitch_pos_pid.calc(gimbal_pitch_target_rad, pitch_fdb, kIntegralPause);
  pitch_spd_pid.calc(pitch_pos_pid.out, gimbal_pitch.speed);

  gimbal_yaw_out_v = yaw_spd_pid.out;
  gimbal_pitch_out_v = pitch_spd_pid.out;

  // ---- 5. 饱和保护：输出贴限幅太久 = 顶住了或在震荡 ----
  const bool saturated = (std::fabs(gimbal_yaw_out_v) >= (kMaxVoltage - 0.5f)) ||
                         (std::fabs(gimbal_pitch_out_v) >= (kMaxVoltage - 0.5f));
  if (saturated) {
    if (!g_sat_active) {
      g_sat_active = true;
      g_sat_since_ms = now;
    }
    else if ((now - g_sat_since_ms) >= kSatTripMs) {
      g_sat_trips++;
      yaw_pos_pid.clear();
      yaw_spd_pid.clear();
      pitch_pos_pid.clear();
      pitch_spd_pid.clear();
      gimbal_yaw_target_rad = yaw_fdb;
      gimbal_pitch_target_rad = pitch_fdb;
      gimbal_yaw_out_v = 0.0f;
      gimbal_pitch_out_v = 0.0f;
      g_input_latched = true;  // 锁定输入：否则摇杆一推又冲进去，变成反复撞击
      g_sat_active = false;
    }
  }
  else {
    g_sat_active = false;
  }

  // ---- 6. 下发（关掉的轴 / 零电压测试一律给 0V）----
  if (kZeroVoltageTest) {
    gimbal_yaw_out_v = 0.0f;
    gimbal_pitch_out_v = 0.0f;
  }
  gimbal_yaw.cmd(kEnableYaw ? gimbal_yaw_out_v : 0.0f);
  gimbal_pitch.cmd(kEnablePitch ? gimbal_pitch_out_v : 0.0f);
  send_voltage();
}

}  // namespace

// [解析] 任务本体：先配 CAN1 与接收中断，再进 1ms 固定节拍。函数不会返回。
extern "C" void gimbal_task(void const * argument)
{
  (void)argument;

  can1.config();  // 全通配过滤器（只有 0x205/0x206 有用），按 ID 分发在下面的回调里
  can1.start();   // 启动 CAN1 并打开 FIFO0 接收中断

  for (;;) {
    gimbal_step();
    osDelay(1);
  }
}

// [解析] CAN1 接收回调：只认两台 GM6020 的 0x205/0x206。
// [解析] 必须"先把 FIFO 取空再判 ID"，反过来写会在总线上出现无关帧时卡死在中断里。
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

/*==============================================================================
 * 上电验证顺序
 *
 * 1) 手扶云台、先不推杆：Live Watch 看 `gimbal_imu_yaw` / `gimbal_imu_pitch` 应在缓慢变化
 *    （陀螺零偏会漂），`g_imu_stale` 必须是 false。若 g_imu_stale=true → BMI088 没数据。
 * 2) 推右摇杆上下：`gimbal_pitch_target_rad` 应变化、`gimbal_imu_pitch` 应跟随。
 *    - 方向反 → 改 kPitchSign；
 *    - 推"上下"时 `gimbal_imu_pitch` 不动而 `imu.roll` 在动 → 把 kPitchFromRoll 改成 true。
 * 3) 推右摇杆左右：`gimbal_imu_yaw` 应连续变化（转过 ±180° 也不能跳变，解卷绕已处理）。
 * 4) 放开手静置 30 秒：应静止；`g_sat_trips` 应为 0，`g_input_latched` 应为 false。
 *
 * 调参顺序（一次只动一个，抖了就退回去）：
 *   a) 速度环：把 kYawPosKp/kPitchPosKp 临时设 0.5，只看速度环是否平稳；
 *      "嗡嗡"或高频抖动 → kYawSpdKp/kPitchSpdKp 从 2 往 1 降。
 *   b) 角度环 Kp：3 → 4 → 5（轻微过冲就退回一格）。
 *   c) 角度环 Kd：0.1 → 0.2 压过冲；出现"沙沙"高频颤就退回。
 *   d) 最后才加 pitch 积分：kPitchPosKi 0 → 0.5 → 1，出现"1~2 秒周期的来回摆"就是到量了。
 *
 * IMU 模式的三个已知限制（原理决定的，不是参数问题）：
 *   - 没有磁力计，`imu.yaw` 会随陀螺零偏缓慢漂移（0.1°/s 量级）：保持朝向只能维持几十秒。
 *   - `imu.pitch` 来自 asin，只有 ±90°；本云台限位 ±51° 够用，将来要过顶得换 pitch_geom。
 *   - IMU 采样比控制节拍晚 1~2 拍，外环刚度天生不如编码器版。
 *============================================================================*/
