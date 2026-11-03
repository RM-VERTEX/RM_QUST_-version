// [解析] 云台任务（验证模型 / 云台板专用）：Yaw = GM6020 ID1(0x205)，Pitch = GM6020 ID2(0x206)，都在 CAN1。
// [解析] 思路照 MAS 云台：摇杆当"速度输入"积分成角度目标 → 角度环 → 速度环 → 电压（串级 PID、输出走 0x1FF）。
// [解析] 与 MAS 的差异：MAS 的角度环反馈用"云台 IMU 欧拉角"；本验证模型的 C 板（含 BMI088）固定在底座上，
// [解析]   量不到云台转过的角度，所以外层反馈用 6020 的绝对编码器角度。将来云台装了自己的 IMU，
// [解析]   只要把 kYawFeedback / kPitchFeedback 换成 imu.yaw / imu.pitch 即可，串级结构不用动。
// [解析] 本板不和底盘板通信，所以本任务独占 CAN1：config/start、发 0x1FF、收 0x205/0x206 全在这个文件里。

#include "cmsis_os.h"

#include <cmath>

#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;  // 定义在 uart_task.cpp

// [解析] GM6020_V = 电压控制（指令帧 0x1FF，与 MAS 一致）；ratio=1.0（直驱）；multi_circle 让 yaw 能连续累计圈数。
// [解析] 这两台电机的对象放在文件作用域（非 static），方便 Keil 的 Live Watch 直接展开看 angle/speed。
sp::RM_Motor gimbal_yaw(1, sp::RM_Motors::GM6020_V, 1.0f);
sp::RM_Motor gimbal_pitch(2, sp::RM_Motors::GM6020_V, 1.0f);

// [解析] 目标角与下发电压：也放文件作用域，Live Watch 里一眼能看到"目标/实际/输出"三者。
float gimbal_yaw_target_rad = 0.0f;    // 期望 yaw 角（编码器系，可连续累计）
float gimbal_pitch_target_rad = 0.0f;  // 期望 pitch 角（编码器系，受限位）
float gimbal_yaw_out_v = 0.0f;         // 最终下发的电压，单位 V
float gimbal_pitch_out_v = 0.0f;
float gimbal_yaw_stick = 0.0f;         // 当前右摇杆水平量（已过死区），中位必须是 0
float gimbal_pitch_stick = 0.0f;       // 当前右摇杆垂直量（已过死区），中位必须是 0
bool g_input_latched = false;          // 顶死锁定：为 true 时不接受摇杆，两杆回中才解锁

// [解析] ===== 诊断量（Live Watch 用，不参与控制）=====
uint32_t g_gimbal_loop_hz = 0;    // 实测控制频率，正常应接近 1000；明显低于 800 说明节拍被拖慢（增益会等效放大）
uint32_t g_yaw_fb_lost = 0;       // Yaw 反馈从"在线"变"掉线"的次数（CAN 丢帧会涨）
uint32_t g_pitch_fb_lost = 0;     // Pitch 同上

namespace
{
/*==================== 帧与量纲 ====================*/
// [解析] GM6020 电压帧 0x1FF 装 ID1~4；本模型只占 ID1/ID2。
constexpr uint16_t kTxId = 0x1FF;

/*==================== 控制参数 ====================*/
constexpr float kDt = 1e-3f;  // 控制周期, 必须与 osDelay(1) 一致

// [解析] 摇杆满偏时"目标角"的变化速率（即操作手能给出的最大转动速度）。
// [解析] MAS 用 Gimbal_LeverSpeedMapRate 换算成约 154°/s(pitch) / 346°/s(yaw)，这里取保守值。
constexpr float kYawLeverRate = 4.0f;    // rad/s ≈ 229°/s
constexpr float kPitchLeverRate = 2.0f;  // rad/s ≈ 115°/s
constexpr float kStickDeadzone = 0.06f;

// [解析] 方向：右推 yaw 顺时针（编码器逆时针为正，故取负）；上推 pitch 抬头（取正）。
// [解析] ★ 实机若方向相反，只改这两个符号，不要改 PID 或摇杆逻辑。
constexpr float kYawSign = -1.0f;
constexpr float kPitchSign = +1.0f;

// [解析] Pitch 机械限位（编码器系，单位 rad）。按验证模型的实际行程改。
constexpr float kPitchMin = -0.9f;  // ≈ -51°
constexpr float kPitchMax = +0.9f;  // ≈ +51°

// [解析] 电压上限：GM6020 满量程 ±24V（GM6020_V 的 cmd() 入参就是 V）。先留 20V 余量。
constexpr float kMaxVoltage = 20.0f;

// [解析] 串级 PID：角度环输出"目标角速度(rad/s)"，速度环输出"电压(V)"。
// [解析] ★ 2026-11-02 实机出现"上电过一会疯狂抖动"，原因是积分 windup + 速度环增益过高，
// [解析]   已把两轴的积分都关掉（Ki=0）、速度环 Kp 8→3、角度环 Kp 8→4、Kd 0.3→0.1。
// [解析]   先把"纯 P + 低增益"跑稳，再按文末的调参顺序逐项加回去。
// [解析] 之前那组 8/8 的值来自理想模型仿真（无延迟、无量化），实机不能直接用。
constexpr float kYawPosKp = 4.0f;    // (rad/s) / rad：1 rad 误差 → 4 rad/s 目标
constexpr float kYawPosKi = 0.0f;    // ★ 起步阶段一律不加积分
constexpr float kYawPosKd = 0.1f;    // 阻尼（sp::PID 的 D 项是"微分先行"，作用于反馈）
constexpr float kYawSpdKp = 3.0f;    // V / (rad/s)

constexpr float kPitchPosKp = 4.0f;
constexpr float kPitchPosKi = 0.0f;  // ★ 确认不抖后，再把 pitch 的 Ki 从 0 慢慢加到 1~4 消重力静差
constexpr float kPitchPosKd = 0.1f;
constexpr float kPitchSpdKp = 3.0f;

constexpr float kMaxSpeedCmd = 6.0f;   // 角度环输出限幅 rad/s（≈344°/s）
constexpr float kPitchMaxIout = 1.5f;  // pitch 角度环积分限幅 rad/s（加 Ki 时才有意义）
constexpr float kPosAlpha = 0.5f;      // 角度环 D 项一阶滤波（减小编码器量化噪声）

// [解析] 积分分离：只有 |P 项输出| 小于这个值（即误差已经不大）才允许积分。
// [解析] 目的是"被限位/被重力顶住时不要攒积分"，这是修掉 windup 的关键。Ki=0 时它不起作用。
constexpr float kIntegralPause = 0.6f;  // 单位 rad/s（P 项输出），约等于 0.15 rad 的误差

// [解析] 饱和保护：输出贴限幅连续超过这个时间，就判定"顶住/震荡"，清积分并把目标拉回当前角。
constexpr float kSatTripMs = 400;

// [解析] ★ 单轴调试开关：把某一轴关掉（该轴恒输出 0V、不发指令），用来确认"到底哪根轴在抖"。
// [解析]   调试顺序建议：先只开 yaw 跑一遍，再只开 pitch 跑一遍，最后两个都开。
// [解析]   关掉的那根轴不会参与"反馈是否在线"的仲裁，所以没接电机也能单独调另一轴。
constexpr bool kEnableYaw = true;
constexpr bool kEnablePitch = true;

// [解析] ★ 二分定位开关（2026-11-02 抖动用）：
// [解析]   true  = 两轴恒发 0V（电机自由，闭环只算不用）。上电后如果**还抖**，说明问题不在我们的环路，
// [解析]           而在机械（齿隙/卡涩/配重）或电机/供电/CAN 本身；如果**不抖了**，说明问题出在环路参数。
// [解析]   false = 正常闭环。
// [解析] 测试时请用手扶住云台，0V 状态下云台是松的会掉下去。
constexpr bool kZeroVoltageTest = false;

/*==================== 诊断计数 ====================*/
uint32_t g_loop_count = 0;        // 当前统计窗口内的循环次数
uint32_t g_hz_window_start = 0;   // 统计窗口起点
bool g_yaw_was_ok = false;        // 上一拍 Yaw 反馈是否在线（用于检测"在线→掉线"跳变）
bool g_pitch_was_ok = false;

/*==================== 对象 ====================*/
// [解析] 本板只跑云台，CAN1 由本任务独占，所以可以放心用 sp::CAN（不会再有两个任务抢 tx_data）。
sp::CAN can1(&hcan1);

// [解析] sp::PID 参数顺序: (dt, kp, ki, kd, max_out, max_iout, alpha)
// [解析] 速度环不用积分（照 MAS）：静差靠角度环消，速度环只做"够快"的内环。
sp::PID yaw_pos_pid(kDt, kYawPosKp, kYawPosKi, kYawPosKd, kMaxSpeedCmd, 0.0f, kPosAlpha);
sp::PID yaw_spd_pid(kDt, kYawSpdKp, 0.0f, 0.0f, kMaxVoltage, 0.0f);
sp::PID pitch_pos_pid(kDt, kPitchPosKp, kPitchPosKi, kPitchPosKd, kMaxSpeedCmd, kPitchMaxIout,
                      kPosAlpha);
sp::PID pitch_spd_pid(kDt, kPitchSpdKp, 0.0f, 0.0f, kMaxVoltage, 0.0f);

bool g_sat_active = false;       // 是否正处于"输出贴限幅"的状态
uint32_t g_sat_since_ms = 0;     // 开始贴限幅的时刻
uint32_t g_sat_trips = 0;        // 触发饱和保护的累计次数（Live Watch 看它涨不涨）

bool g_inited = false;  // 是否已用当前角度初始化过目标

/*==================== 内部函数 ====================*/
float deadzone(float v) { return (std::fabs(v) < kStickDeadzone) ? 0.0f : v; }

// [解析] 一帧 0x1FF 装 2 台 6020 的电压：各写自己那 2 字节（ID1→[0..1]、ID2→[2..3]），再发一次。
void send_voltage()
{
  gimbal_yaw.write(can1.tx_data);
  gimbal_pitch.write(can1.tx_data);
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) can1.send(kTxId);
}

// [解析] 失联/异常的统一出口：清 PID 状态（否则积分残留会在恢复瞬间猛冲）并下发 0V。
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

// [解析] 上电第一次拿到反馈时，把目标角对齐到当前角，避免"一上电就往 0°冲"。
// [解析] Pitch 这里不做限幅：若静止角本来就在行程外，限幅会让目标瞬间跳到边界、电机立刻动；
// [解析]   不加限幅则上电零误差，之后的摇杆增量走 kPitchMin/Max 限幅，能把云台慢慢拉回行程内。
void init_targets()
{
  gimbal_yaw_target_rad = gimbal_yaw.angle;
  gimbal_pitch_target_rad = gimbal_pitch.angle;
  g_inited = true;
}

// [解析] 一个控制周期：摇杆积分成目标角 → 角度环 → 速度环 → 电压。
void gimbal_step()
{
  const uint32_t now = osKernelSysTick();

  // ---- 0. 诊断：实测控制频率（500ms 窗口）----
  // [解析] 频率如果明显低于 1000Hz，说明 osDelay(1) 的实际周期被拖长了；
  // [解析]   而 PID 的 dt 是写死 1e-3 的，等效于把增益放大了同样的倍数 → 会抖。
  g_loop_count++;
  if ((now - g_hz_window_start) >= 500U) {
    g_gimbal_loop_hz = g_loop_count * 2U;  // 0.5s 窗口 → ×2 = Hz
    g_loop_count = 0;
    g_hz_window_start = now;
  }

  // ---- 1. 反馈仲裁：在用的电机只要有一个没反馈就全部归零（看不见就不动）----
  const bool yaw_ok = (!kEnableYaw) || gimbal_yaw.is_alive(now);
  const bool pitch_ok = (!kEnablePitch) || gimbal_pitch.is_alive(now);

  // [解析] 诊断：统计"在线→掉线"的次数。这个数一直涨就说明 CAN 反馈在断续，
  // [解析]   那么 stop_gimbal() 会每隔一会把电压清 0，云台表现为抽动（而不是 PID 振荡）。
  if (g_yaw_was_ok && !yaw_ok) g_yaw_fb_lost++;
  if (g_pitch_was_ok && !pitch_ok) g_pitch_fb_lost++;
  g_yaw_was_ok = yaw_ok;
  g_pitch_was_ok = pitch_ok;

  if (!yaw_ok || !pitch_ok) {
    g_inited = false;  // 重新上线后重新对齐目标
    stop_gimbal();
    return;
  }
  if (!g_inited) init_targets();

  // ---- 2. 摇杆 → 目标角（摇杆是"速度"输入，积分成角度）----
  // [解析] ★ 用**右摇杆**（ch_rh 水平 / ch_rv 垂直）：DJI 遥控器在各手模式下右摇杆都是弹簧回中的。
  // [解析]   不要用 ch_lv：它是"左垂直"，美国手下是**油门，弹簧不回中**，会停在非零位置 →
  // [解析]   目标角一直朝一个方向积分 → 撞限位 → 顶死 → 就是 2026-11-02 那次"疯狂抖动"。
  float yaw_stick = 0.0f;
  float pitch_stick = 0.0f;
  if (remote.is_alive(now)) {
    yaw_stick = deadzone(sp::limit_min_max(remote.ch_rh, -1.0f, 1.0f));
    pitch_stick = deadzone(sp::limit_min_max(remote.ch_rv, -1.0f, 1.0f));
  }
  gimbal_yaw_stick = yaw_stick;    // 给 Live Watch 看：摇杆中位必须读到 0.000
  gimbal_pitch_stick = pitch_stick;

  if (g_input_latched) {
    // [解析] 顶死锁定期间：不接受摇杆，目标退回当前角（停止死顶）。
    // [解析] 必须**两杆都回中**才解锁 —— 这样操作手一定会意识到"它在等松手"，而不是继续抖。
    gimbal_yaw_target_rad = gimbal_yaw.angle;
    gimbal_pitch_target_rad = gimbal_pitch.angle;
    if (std::fabs(yaw_stick) < kStickDeadzone && std::fabs(pitch_stick) < kStickDeadzone) {
      g_input_latched = false;
    }
  }
  else {
    gimbal_yaw_target_rad += kYawSign * yaw_stick * kYawLeverRate * kDt;
    gimbal_pitch_target_rad += kPitchSign * pitch_stick * kPitchLeverRate * kDt;
    gimbal_pitch_target_rad = sp::limit_min_max(gimbal_pitch_target_rad, kPitchMin, kPitchMax);
  }
  // 遥控器失联时保持目标角不变：电机仍然闭环保持姿态，不会自己掉下来

  // ---- 3. 串级 PID：角度环（编码器角）→ 速度环（转子 rad/s）→ 电压 ----
  // [解析] 角度环用带"积分分离"的重载：|P 项| 大于阈值时不积分，被限位/被重力顶住时不会攒积分。
  const float yaw_fdb = gimbal_yaw.angle;
  const float pitch_fdb = gimbal_pitch.angle;
  yaw_pos_pid.calc(gimbal_yaw_target_rad, yaw_fdb, kIntegralPause);
  yaw_spd_pid.calc(yaw_pos_pid.out, gimbal_yaw.speed);
  pitch_pos_pid.calc(gimbal_pitch_target_rad, pitch_fdb, kIntegralPause);
  pitch_spd_pid.calc(pitch_pos_pid.out, gimbal_pitch.speed);

  gimbal_yaw_out_v = yaw_spd_pid.out;
  gimbal_pitch_out_v = pitch_spd_pid.out;

  // ---- 4. 饱和保护：输出贴限幅太久 = 顶住了或在震荡 ----
  // [解析] 正常快速转动只会短暂贴限幅（几十 ms）；连续 400 ms 贴住说明目标够不到
  // [解析]   （撞机械限位/被重力卡住/方向搞反），此时必须清积分并把目标拉回当前角，否则就是死顶。
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
      gimbal_yaw_target_rad = yaw_fdb;      // 目标退回当前角，停止"死顶"
      gimbal_pitch_target_rad = pitch_fdb;
      gimbal_yaw_out_v = 0.0f;
      gimbal_pitch_out_v = 0.0f;
      g_input_latched = true;  // ★ 锁定输入：否则摇杆一推又冲进去，变成 1 秒一次的反复撞击
      g_sat_active = false;
    }
  }
  else {
    g_sat_active = false;
  }

  // ---- 5. 下发（被 kEnableYaw/kEnablePitch 关掉的轴恒给 0V）----
  // [解析] kZeroVoltageTest 是最高优先级覆盖：它必须放在所有计算之后，保证最终一定是 0V。
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
 * 调参顺序（务必按这个顺序，一次只动一个参数，抖了就退回去）
 *
 * 0) 先确认机械没问题：断电手动转一圈，确认没有卡涩/线束干涉；确认 kPitchMin/Max
 *    是**实测**的机械行程（现在是 ±0.9 rad 占位值，编码器系，见文件头说明）。
 *
 * 1) 速度环（内环）先单独稳：把 kYawPosKp / kPitchPosKp 临时改成 0.5（几乎只剩速度环），
 *    推摇杆看是否平稳跟速。若"嗡嗡"或高频抖动 → 把 kYawSpdKp / kPitchSpdKp 从 3 往 1 降。
 *    内环不稳，外环怎么调都会抖。
 *
 * 2) 角度环 Kp：把 kYawPosKp / kPitchPosKp 从 4 往上加（4→6→8），直到出现轻微过冲就退回一格。
 *    此时 Ki 仍为 0。
 *
 * 3) 角度环 Kd：加 kYawPosKd / kPitchPosKd（0.1→0.2）压过冲。
 *    D 项吃编码器量化噪声，加太多会听到齿轮"沙沙"声或高频颤 → 降回去。
 *
 * 4) 最后才加积分（只加 pitch）：kPitchPosKi 从 0 → 0.5 → 1 → 2，边加边看：
 *    - 静差消失得差不多就停；
 *    - 若到位后出现"低频来回摆"（周期 0.5~2 s）→ 积分过大，减半；
 *    - kPitchMaxIout=1.5 配合 kIntegralPause=0.6 已经防了大部分 windup，但别指望它万能。
 *
 * 5) 观察 g_sat_trips：正常情况下它应该一直是 0。
 *    如果它持续增长，说明某个轴的指令长期够不到（撞限位/方向反/机械卡住），
 *    这时**不要调 PID**，先解决方向或限位问题。
 *============================================================================*/
