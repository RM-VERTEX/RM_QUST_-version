// [解析] 云台控制链路 · 第二步：IMU 角速度内环（开�?�?单速度环）。Yaw = GM6020 ID1，Pitch = ID2，都�?CAN1�?
// [解析] 结构（参照狼�?2022）：摇杆 �?目标角速度 �?速度环（反馈 = C �?IMU 角速度）→ 电压 �?0x1FF�?
// [解析] 内环为什么用 IMU 角速度而不是电机转速：陀螺测的是"云台在世界系的真实转�?�?
// [解析]   不受齿隙/联轴器间隙影响，且自带高频阻尼，比编码器差分更容易稳�?
// [解析] �?参数取最保守的一档（Kp=1.5 V/(rad/s)、限�?6V、目标转�?0.8rad/s）：先求"能动且不�?�?
// [解析] �?符号安全：速度环是负反馈，"反馈符号"�?电压→转�?必须一致。若一推杆就朝一个方向加速跑飞，
// [解析]   �?kRateLoopInvert 改成 true。另�?跑飞保护"兜底（kRateLimitTrip）�?

#include "cmsis_os.h"

#include <cmath>

#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;   // 定义�?uart_task.cpp�? kHz 解析 DBus�?
extern sp::Mahony imu;    // 定义�?imu_task.cpp�? kHz 更新姿态与角速度�?
extern sp::BMI088 bmi088;  // 定义�?imu_task.cpp（机体角速度，已�?r_ab 转到云台系）

// [解析] GM6020_V = 电压控制：指令帧 0x1FF，ID1→data[0..1]、ID2→data[2..3]，大�?int16�?
sp::RM_Motor gimbal_yaw(1, sp::RM_Motors::GM6020_V, 1.0f);
sp::RM_Motor gimbal_pitch(2, sp::RM_Motors::GM6020_V, 1.0f);

// [解析] ===== Live Watch =====
float gimbal_yaw_out_v = 0.0f;     // 本拍发给 Yaw 的电�?
float gimbal_pitch_out_v = 0.0f;   // 本拍发给 Pitch 的电�?
float gimbal_yaw_stick = 0.0f;     // 右摇杆水平量（已过死区）
float gimbal_pitch_stick = 0.0f;   // 右摇杆垂直量
float gimbal_yaw_angle_deg = 0.0f;    // 实测 yaw 角（度，连续�?
float gimbal_pitch_angle_deg = 0.0f;  // 实测 pitch 角（度，低头为正�?
float gimbal_yaw_target_deg = 0.0f;   // 目标 yaw 角（度）
float gimbal_pitch_target_deg = 0.0f; // 目标 pitch 角（度）
float gimbal_yaw_pos_err = 0.0f;      // 位置环误差（度）
float gimbal_pitch_pos_err = 0.0f;    // 位置环误差（度）
float gimbal_yaw_rate_cmd = 0.0f;  // 目标角速度 rad/s
float gimbal_pitch_rate_cmd = 0.0f;
float gimbal_yaw_rate = 0.0f;      // 实测角速度（四元数投影 + 低通）rad/s
float gimbal_pitch_rate = 0.0f;
float gimbal_yaw_rate_raw = 0.0f;    // 实测角速度（未滤波，四元数投影�?
float gimbal_pitch_rate_raw = 0.0f;
float gimbal_yaw_rate_euler = 0.0f;  // 对照用：旧的欧拉角速率 imu.vyaw（万向锁时会发散，仅诊断�?
float gimbal_pitch_rate_euler = 0.0f;
float gimbal_pitch_geom_deg = 0.0f;  // 几何 pitch 角（±180°，低头为正），用来做安全限位
float gimbal_yaw_rate_err = 0.0f;    // 速度环误差（已含极性）
float gimbal_pitch_rate_err = 0.0f;
bool g_pitch_limit_tripped = false;  // pitch 超过安全角，已切断输�?

// [解析] ===== 飞行记录仪（GDB 调参用）=====
// [解析] 目的：把"推杆瞬间"的完整波形存�?RAM，事后用 GDB dump 出来分析�?
// [解析]   不依�?Live Watch �?1s 刷新（那根本看不�?100ms 级的瞬态）�?
// [解析] 触发：摇杆量首次超过 kRecTrigger 时开始，记录 kRecLen 个周期（1kHz �?512ms）后停止�?
// [解析] 用法：推一次杆 �?GDB �?dump rec_buf �?本地分析超调/振荡/稳态误差�?
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
volatile uint32_t g_rec_state = 0;  // 0=等待触发 1=录制�?2=已录完（可读�?3=被强制重�?
uint32_t g_rec_n = 0;               // 已记录样本数
float g_rec_tick = 0.0f;            // 录制内相对时�?ms)
uint32_t g_rec_reset_req = 0;       // GDB �?1 �?清空重录

// [解析] ===== 定压探针的极值锁存（Live Watch 刷不过来时用这个看结果）=====
// [解析] 每次 "+V 窗口" 内记�?pitch 转速的最�?最小值，窗口结束�?*保持不动**�?
// [解析]   所以转完停下来再看也来得及：看 g_probe_max / g_probe_min 的符号就够判断极性�?
float g_probe_max = 0.0f;   // 最近一�?+V 窗口�?vpitch 的最大�?
float g_probe_min = 0.0f;   // 最近一�?+V 窗口�?vpitch 的最小�?
bool g_probe_on = false;    // 当前是否处于 +V 窗口
uint32_t g_gimbal_loop_hz = 0;       // 实测循环频率，正�?�?000
uint32_t g_tx_drop_count = 0;        // 邮箱满导致没发出去的次数
bool g_imu_stale = false;            // BMI088 数据连续多拍没变�?
bool g_rate_runaway = false;         // 跑飞保护已触发（摇杆回中才恢复）

namespace
{
/*==================== 参数（最保守一档）====================*/
constexpr float kDt = 1e-3f;  // 控制周期，必须与 osDelay(1) 一�?

// [解析] 目标角速度上限：★ 分轴 —�?yaw 可以快，pitch 机械受限且受重力影响，单独压低�?
// [解析]   2026-11-04 实测：两轴共�?4.0 rad/s �?pitch 抖动，所以把 pitch 收回�?2.0�?
constexpr float kYawMaxRate = 4.0f;    // rad/s �?229°/s（一�?1.6s�?
constexpr float kPitchMaxRate = 1.5f;  // rad/s �?86°/s（抖动未解决前先降速）

/*==================== 外环（位置环）参�?====================*/
// [解析] 位置�?P：输�?�?，输�?rad/s"（目标角速度）�?
// [解析]   1° 误差时：yaw �?4.0·0.01745 = 0.07 rad/s�?° �?0.35 rad/s —�?所以死区不能大�?
// [解析]   否则位置环输出永远够不到内环门槛（这就是为什么下面把内环死区调小了）�?
constexpr float kYawPosKp = 4.0f;    // (rad/s) / rad
constexpr float kPitchPosKp = 3.0f;

// [解析] 位置环死区（度）：误差小于它就当�?0，不去追静摩�?—�?这是 MAS 那个 ±1° 的核心�?
constexpr float kPosDeadzoneDeg = 1.0f;

// [解析] 摇杆积分速率（度/秒）：推满杆时目标角每秒变化多少�?
constexpr float kYawLeverDegPerS = 240.0f;
constexpr float kPitchLeverDegPerS = 150.0f;

// [解析] pitch 软限位（度，**低头为正**）：目标角被夹在这个范围内�?
// [解析]   �?这是"目标限幅"而不�?切输�?，所以推到边界只是推不动，不会像之前那样把云台弄失控�?
// [解析]   必须按你的机构实际行程按需修改（现在给 ±35° 是通用占位值）�?
constexpr float kPitchSoftMinDeg = -35.0f;  // 抬头极限（负�?
constexpr float kPitchSoftMaxDeg = +35.0f;  // 低头极限（正�?

// [解析] 速度环增益：0.8rad/s 误差 �?1.2V。刻意压低，宁可慢也不要抖�?
// [解析] �?分轴给：pitch 曾出�?一动就疯狂抖动"，若确诊是环路太硬，单独�?pitch 压低即可�?
constexpr float kYawRateKp = 3.2f;    // V / (rad/s)
// [解析] 前馈系数：电压 / 目标转速，由实测标定（0.85V ↔ 0.5rad/s → 1.7 V/(rad/s)）。
// [解析]   ★ 前馈不参与环路，不影响稳定性；调大只会让"同样指令更用力"，过大会过冲，可单独微调。
constexpr float kYawRateFF = 1.7f;    // V / (rad/s)
constexpr float kPitchRateFF = 1.7f;
constexpr float kPitchRateKp = 0.4f;  // V / (rad/s)：降速同步降增益

// [解析] 积分：★ 2026-11-04 仿真实测，pitch **必须**开速度环积分，否则�?P 串级顶不住重力：
// [解析]   �?P 时稳态需要一个永久的"速率误差"才能产生保持电压 �?云台就匀速下坠（仿真�?10s �?141°）�?
// [解析]   加在速度环（不是位置环）：位置环输出已饱和在 kPitchMaxRate，它的积分涨了也没用�?
// [解析]   仿真对比（带重力�?0° 阶跃）：纯P �?�?141°；速度�?Ki=1.0 �?残差 0.9°、无振荡�?
constexpr float kYawRateKi = 0.0f;     // yaw 无重力，不加（避免积�?windup�?
constexpr float kPitchRateKi = 0.5f;   // �?pitch 抗重力，必须�?
constexpr float kPitchRateMaxIout = 3.0f;  // 积分输出限幅 V：够顶重力，又不至于把电压顶�?

// [解析] 电压硬限幅。★ 分轴：pitch 一旦失控最容易撞限位，先给它一个更小的上限�?V）兜住�?
constexpr float kYawMaxVoltage = 14.0f;
constexpr float kPitchMaxVoltage = 8.0f;  // 降速同步降电压上限

// [解析] 误差死区：|目标-实测| 小于它就不动作�?
// [解析] �?这是"轻碰就高频抖"�?*第一个旋�?*：死区必须明显大于陀螺噪声（BMI088 静止噪声�?0.02~0.05 rad/s），
// [解析]   否则静止时误差在噪声里反复穿�?0，环路一直点电机 �?高频颤�?
// [解析] �?分轴死区：pitch 轴惯量小、易被噪声激励，死区给得�?yaw 大得多（0.35 �?20°/s）�?
// [解析]   �?加了位置环之后这两个值必须调小：位置环输�?= Kp·角度误差�?
// [解析]     内环死区太大时位置环的小误差根本推不动电机（会变�?0.4/1.2 �?19° �?整块死区"）�?
// [解析]     "不去追小误差"的职责交给位置环�?kPosDeadzoneDeg，内环只需要挡住噪声�?
constexpr float kYawRateDeadzone = 0.01f;    // rad/s
constexpr float kPitchRateDeadzone = 0.01f;  // rad/s：pitch 略大一点，压它的陀螺噪�?

// [解析] 反馈一阶低通：alpha 越大越平滑、相位滞后越大�?
// [解析] �?这是第二个旋钮：0.85 �?6.3ms 时间常数，把陀螺噪声压掉一大截（代价是响应变钝一点）�?
constexpr float kRateAlpha = 0.5f;

// [解析] 积分分离阈值（单位：伏特，比较 |P 项输出|）。只�?Ki>0 的轴起作用�?
constexpr float kIntegralPause = 1.5f;

// [解析] 摇杆死区；上电静默�?
constexpr float kStickDeadzone = 0.06f;
constexpr uint32_t kStartupDelayMs = 1000;

// [解析] 方向�?026-11-03 实测校正）：
// [解析]   �?+1/+1 时：左推 �?yaw 顺时针、上�?�?俯下（两个都反）
// [解析]   现在 -1/-1：右�?�?yaw 顺时针、上�?�?抬头
// [解析] �?这两个符号只影响"摇杆方向"，不影响环路极性；环路极性由 kYawRateInvert/kPitchRateInvert 管�?
constexpr float kYawSign = -1.0f;
constexpr float kPitchSign = -1.0f;

// [解析] �?分轴负反馈极性开关：某轴"推杆后越转越�?撞限位抖"就是这个轴反了，单独翻它�?
// [解析]   两轴的正转方向与 C �?+x/+y 轴是相互独立的，所以必须分开判断（同一条总线上也常见一轴对一轴反）�?
constexpr bool kYawRateInvert = false;
// [解析] 2026-11-03 第一轮：pitch 抖动 �?�?true 后变�?一动就�?180°"�?
// [解析]   说明不是极性，而是**反馈根本没读�?pitch 转�?*（开环跑飞）。等轴映射查清后再定这个值�?
constexpr bool kPitchRateInvert = false;

// [解析] ★★ IMU 轴映射探测模式：两轴恒输�?0V、不进速度环，只把 IMU 的角速度实时显示出来�?
// [解析]   2026-11-03 已用它确认：�?pitch 轴转只有 imu.vpitch 变（俯头正、仰头负）→ 轴映射正确、r_ab 用单位阵�?
constexpr bool kImuProbeOnly = false;

// [解析] ★★ 0V 裸测开关（本次为 true）：两轴恒输出 0V，其他逻辑照跑、记录仪照记。
// [解析]   目的：分清"抖动是环路自己造的"还是"陀螺测到了真实的机械振动"。
// [解析]   现象判读（手推/轻碰云台时）：
// [解析]     - 若 0V 下 gyro 仍然出现同样的 35Hz → 纯机械/电气振动，调 PID 永远治不好，去查结构/电机；
// [解析]     - 若 0V 下 gyro 很干净 → 抖动完全是环路造成的，回来继续调增益/滤波。
// [解析]   测完务必改回 false，否则云台永远不出力。
constexpr bool kOutputTestZero = false;

// [解析] ★★ 极性探针（当前启用）：>0 时进�?定压测试" —�?不闭环，pitch �?1.5s 交替�?+kProbeVoltage / 0V�?
// [解析]   用法：看 gimbal_pitch_rate_raw 的符号，就能直接判定速度环该怎么接：
// [解析]     V>0 �?vpitch �?*�?*  �?电压�?= 俯头 �?反馈取正，kPitchRateInvert = false
// [解析]     V>0 �?vpitch �?*�?*  �?电压�?= 仰头 �?反馈要取反，kPitchRateInvert = true
// [解析]   这个实验不闭环、电压只�?1.5V，绝对不会跑飞；测完改回 0�?
constexpr float kProbeVoltage = 0.0f;  // 0 = 正常闭环（当前）�?0 = 定压极性探�?
constexpr uint32_t kProbeOnMs = 2500;  // 探针每段 2.5s

// [解析] 保留旧名字做兼容（全局翻转会对两轴同时生效，排查时用）�?
constexpr bool kRateLoopInvert = false;

// [解析] 跑飞保护：实测角速度超过这个值并持续 200ms �?断电锁定，摇杆回中恢复�?
// [解析] �?阈值必须显著大�?kMaxRate，否则正常快速转动会被误判成跑飞�?
constexpr float kRateLimitTrip = 4.0f;  // rad/s �?687°/s（kMaxRate=4.0 �?3 倍）
constexpr uint32_t kRateTripMs = 200;

// [解析] 陀螺投影没�?1/cos 发散问题，但保留一个物理上限防止异常值�?
// [解析] �?必须 > kRateLimitTrip，否则跑飞保护的判据永远达不到（反馈先被钳住了）�?
constexpr float kRateMeasLimit = 10.0f;  // rad/s（GM6020 空载 33 rad/s，留足余量）

// [解析] pitch 安全限位开关（当前**关闭**，用于观察万向锁附近的原始现象）�?
// [解析]   为什么关：切断输�?电机自由，云台会翻过垂直后永久无法解除锁定（详见 gimbal_step 里的注释）�?
constexpr bool kPitchSafeLimitEnable = false;

// [解析] pitch 安全限位：几�?pitch 角超过它就直接切断输出�?
// [解析]   �?pitch_geom（�?80°）而不�?pitch（asin，只�?±90°），否则过了垂直之后 asin �?折回�?看不出超限�?
constexpr float kPitchSafeLimitDeg = 70.0f;

constexpr uint16_t kTxId = 0x1FF;

/*==================== 对象 ====================*/
// [解析] 速度环：默认�?P（Ki=0）。pitch 若受重力影响严重，可�?kPitchRateKi 打开做积分�?
// [解析] max_iout 只在 Ki>0 时起作用，用来防止积分把电压顶到限幅�?
sp::PID yaw_rate_pid(kDt, kYawRateKp, kYawRateKi, 0.0f, kYawMaxVoltage, 3.0f);
sp::PID pitch_rate_pid(kDt, kPitchRateKp, kPitchRateKi, 0.0f, kPitchMaxVoltage,
                       kPitchRateMaxIout);

// [解析] 外环（位置环）：输入度、输�?rad/s；输出限幅就是内环的目标转速上限�?
sp::PID yaw_pos_pid(kDt, kYawPosKp, 0.0f, 0.0f, kYawMaxRate, 0.0f);
sp::PID pitch_pos_pid(kDt, kPitchPosKp, 0.0f, 0.0f, kPitchMaxRate, 0.0f);

/*==================== 状�?====================*/
uint32_t g_loop_count = 0;
uint32_t g_hz_window_start = 0;
bool g_started = false;
uint32_t g_start_ms = 0;

// IMU 是否在更新：四元数连续多拍不变就判定没数�?
float g_q_last[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint32_t g_q_same_count = 0;

// 反馈低通状�?
float g_yaw_rate_f = 0.0f;
float g_pitch_rate_f = 0.0f;
bool g_filter_ready = false;

// 跑飞保护状�?
bool g_runaway_active = false;
uint32_t g_runaway_since = 0;

// 外环（位置环）状�?
bool g_angle_inited = false;       // 目标角是否已对齐过当前角�?
bool g_yaw_angle_ready = false;    // yaw 解卷绕是否已初始�?
float g_yaw_angle_deg_cont = 0.0f; // 连续（可�?±180°）的 yaw 角度
float g_yaw_angle_raw_last = 0.0f; // 上一�?imu.yaw（弧度）

/*==================== 内部函数 ====================*/
float stick_of(float raw)
{
  const float v = sp::limit_min_max(raw, -1.0f, 1.0f);
  return (std::fabs(v) < kStickDeadzone) ? 0.0f : v;
}

// [解析] BMI088 是否在更新（bmi088.init() 找不到芯片会卡在重试里，那时 q 永远是初值）�?
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

// [解析] 一阶低通：alpha 越大越平滑�?kHz �?alpha=0.7 的时间常数约 3ms�?
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

// [解析] 用四元数�?*世界竖直�?*转到机体系：v_body = q* �?v_world �?q（q 是机体→世界的姿态四元数，wxyz）�?
// [解析] 只做这一件事，所以内联实�?—�?sp::Mahony 里那�?quaternion_frame_transform_for_mahony �?private�?
// [解析] 与中间件里同一套公式：v_out = v_in + 2·cross(q_vec, cross(q_vec, v_in) + w·v_in)�?
void world_z_to_body(const float q[4], float out[3])
{
  const float w = q[0];
  const float x = -q[1];  // 取共轭：world �?body
  const float y = -q[2];
  const float z = -q[3];

  const float vx = 0.0f, vy = 0.0f, vz = 1.0f;  // 世界竖直�?

  const float tx = y * vz - z * vy + w * vx;
  const float ty = z * vx - x * vz + w * vy;
  const float tz = x * vy - y * vx + w * vz;

  out[0] = vx + 2.0f * (y * tz - z * ty);
  out[1] = vy + 2.0f * (z * tx - x * tz);
  out[2] = vz + 2.0f * (x * ty - y * tx);
}

// [解析] yaw 角解卷绕：imu.yaw �?atan2 出来的，只在 (-π, π]，绕一圈会突跳 2π�?
// [解析]   这里�?本帧与上帧之�?折算�?(-π, π] 再累加，得到可超�?±180° 的连续角（度）�?
float update_yaw_angle_deg(float raw_rad)
{
  if (!g_yaw_angle_ready) {
    g_yaw_angle_deg_cont = raw_rad * 57.29577951f;
    g_yaw_angle_raw_last = raw_rad;
    g_yaw_angle_ready = true;
    return g_yaw_angle_deg_cont;
  }
  const float delta_rad = sp::limit_angle(raw_rad - g_yaw_angle_raw_last);
  g_yaw_angle_raw_last = raw_rad;
  g_yaw_angle_deg_cont += delta_rad * 57.29577951f;
  return g_yaw_angle_deg_cont;
}

// [解析] 把两台电机的 2 字节指令拼进一�?0x1FF 并发出�?
// [解析] 不用 sp::CAN::send()：它的帧头是函数�?static，和 chassis_task 的发送共用会被抢占写坏�?
void send_frame()
{
  uint8_t frame[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  gimbal_yaw.write(frame);    // ID1 �?frame[0..1]
  gimbal_pitch.write(frame);  // ID2 �?frame[2..3]

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

// [解析] 统一出口：清 PID、下�?0V。任何异常都走这里�?
void stop_gimbal()
{
  yaw_rate_pid.clear();
  pitch_rate_pid.clear();
  yaw_pos_pid.clear();
  pitch_pos_pid.clear();
  g_angle_inited = false;  // 恢复后目标角重新对齐当前姿态，不会突然�?
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

  // ---- 诊断：实测循环频率（500ms 窗口�?---
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

  // ---- IMU 健康检查：没有角速度就没法做速度环，一律断�?----
  update_imu_stale();
  if (g_imu_stale) {
    stop_gimbal();
    return;
  }

  // ---- 反馈：四元数投影�?关节角速度"（彻底避开欧拉角的万向节死锁）----
  // [解析] �?为什么不能直接用 imu.vyaw / imu.vpitch�?
  // [解析]   vyaw = wy·sin(roll)/cos(pitch) + wz·cos(roll)/cos(pitch)，pitch→�?0° �?cos(pitch)�? �?发散�?
  // [解析]   实测现象：pitch 转到垂直，yaw 电机突然猛扭、随�?yaw 失控，�?pitch 还能�?—�?正是这个式的 1/cos�?
  // [解析] �?正确做法：关节角速度 = 机体角速度�?该关节转�?上的投影，任何姿态都有定义：
  // [解析]   - yaw 关节�?= 世界竖直�?z；把它用四元数转到机体系得到 z_body，再与陀螺点乘；
  // [解析]   - pitch 关节�?= 机体�?y 轴（�?y 转不会改�?y 自身方向），直接�?gyro[1]�?
  // [解析]   姿态水平时两者分别退化为 gyro[2]、gyro[1]，与旧的 vyaw/vpitch 同号 �?极性和增益都不用改�?
  float z_in_body[3] = {0.0f, 0.0f, 0.0f};
  world_z_to_body(imu.q, z_in_body);

  const float yaw_rate_q =
    bmi088.gyro[0] * z_in_body[0] + bmi088.gyro[1] * z_in_body[1] + bmi088.gyro[2] * z_in_body[2];
  const float pitch_rate_q = bmi088.gyro[1];

  const float yaw_raw = sp::limit_min_max(yaw_rate_q, -kRateMeasLimit, kRateMeasLimit);
  const float pitch_raw = sp::limit_min_max(pitch_rate_q, -kRateMeasLimit, kRateMeasLimit);

  gimbal_yaw_rate_raw = yaw_raw;
  gimbal_pitch_rate_raw = pitch_raw;
  gimbal_yaw_rate_euler = imu.vyaw;    // 对照：万向锁附近它会炸，仅用于诊�?
  gimbal_pitch_rate_euler = imu.vpitch;
  gimbal_pitch_geom_deg = imu.pitch_geom * 57.29577951f;

  // ---- 安全限位（可关）：pitch 接近垂直就切�?----
  // [解析] �?2026-11-04 实测发现这个策略有缺陷：切断=0V=电机自由，云台会在重力下翻过垂直�?
  // [解析]   之后 pitch_geom 回不到解除阈�?�?**永久锁定，正反转都不响应**�?
  // [解析]   所以先整体关掉（kPitchSafeLimitEnable = false），用来观察万向锁附近的原始现象�?
  // [解析]   将来要恢复，应该改成"只在朝超限方向运动时限幅"，不要切断全部扭矩�?
  if (kPitchSafeLimitEnable) {
    const bool pitch_over = std::fabs(gimbal_pitch_geom_deg) > kPitchSafeLimitDeg;
    if (pitch_over) {
      g_pitch_limit_tripped = true;
    }
    else if (std::fabs(gimbal_pitch_geom_deg) < (kPitchSafeLimitDeg - 5.0f)) {
      g_pitch_limit_tripped = false;  // 5° 回差，避免在边界反复�?
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

  // [解析] 探测模式：两�?0V，不进环�?—�?用手转云台，看三个角速度哪个跟着动�?
  if (kImuProbeOnly) {
    gimbal_yaw_rate_cmd = 0.0f;
    gimbal_pitch_rate_cmd = 0.0f;
    gimbal_yaw_stick = 0.0f;
    gimbal_pitch_stick = 0.0f;
    stop_gimbal();
    return;
  }

  // [解析] 极性探针：定压、不闭环。pitch 每隔 kProbeOnMs 交替�?+kProbeVoltage �?0 之间切换�?
  // [解析]   这样�?gimbal_pitch_rate_raw 的符号就能判定电压与转速的对应关系�?
  if (kProbeVoltage > 0.0f) {
    const bool on = ((now / kProbeOnMs) % 2U) == 0U;
    const float v = on ? kProbeVoltage : 0.0f;

    // [解析] 窗口开始时清空极值，窗口内持续更新；窗口结束后保持不变（锁存），方便事后查看�?
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

  // ---- 角度反馈（外环用�?---
  // [解析] yaw �?imu.yaw 解卷绕（atan2 只有 ±π，绕圈会跳）；pitch 直接�?imu.pitch（asin，�?0° 内够用）�?
  // [解析] 注意：到 pitch=±90° 附近 yaw 本身没有意义（万向锁），但正常行程用不到那里�?
  const float yaw_deg = update_yaw_angle_deg(imu.yaw);
  const float pitch_deg = imu.pitch * 57.29577951f;
  gimbal_yaw_angle_deg = yaw_deg;
  gimbal_pitch_angle_deg = pitch_deg;

  // ---- 摇杆 ----
  float yaw_stick = 0.0f;
  float pitch_stick = 0.0f;
  if (remote.is_alive(now)) {  // 遥控器失�?�?摇杆�?0，目标角保持不变（云台原地守住）
    yaw_stick = stick_of(remote.ch_rh);
    pitch_stick = stick_of(remote.ch_rv);
  }
  gimbal_yaw_stick = yaw_stick;
  gimbal_pitch_stick = pitch_stick;

  // ---- 外环：摇杆积分成"目标角度" ----
  // [解析] 目标角在**上电/恢复时对齐当前角�?*，所以不会一上电就往某个角冲�?
  if (!g_angle_inited) {
    gimbal_yaw_target_deg = yaw_deg;
    gimbal_pitch_target_deg = pitch_deg;
    yaw_pos_pid.clear();
    pitch_pos_pid.clear();
    g_angle_inited = true;
  }
  gimbal_yaw_target_deg += kYawSign * yaw_stick * kYawLeverDegPerS * kDt;
  gimbal_pitch_target_deg += kPitchSign * pitch_stick * kPitchLeverDegPerS * kDt;
  // [解析] pitch 软限位（这里�?*目标�?*限幅，不是切输出）—�?推杆到边界就推不动了，但不会失控�?
  gimbal_pitch_target_deg =
    sp::limit_min_max(gimbal_pitch_target_deg, kPitchSoftMinDeg, kPitchSoftMaxDeg);

  // ---- 外环 PID：角度误�?�?目标角速度 ----
  // [解析] ★★ 单位必须统一�?*弧度**：增益是按摩度整定的�?rad/s)/rad）�?
  // [解析]   2026-11-04 踩过的坑：把"�?直接喂给 PID，Kp=3 就变�?3 rad/s �?1 �?= 172 rad/s �?rad�?
  // [解析]     于是 1.75° 误差就打�?1.5 rad/s �?位置环变�?±1.5 的方波（bang-bang），
  // [解析]     内环追方�?�?冲过头、反�?�?表现�?~88Hz 高频�?+ 稳态差 2.6°�?
  // [解析]   死区 kPosDeadzoneDeg 仍是度（配置直观），进环前换算成弧度�?
  const float deg2rad = 0.0174532925f;

  const float yaw_pos_err = gimbal_yaw_target_deg - yaw_deg;
  const float pitch_pos_err = gimbal_pitch_target_deg - pitch_deg;
  gimbal_yaw_pos_err = yaw_pos_err;
  gimbal_pitch_pos_err = pitch_pos_err;

  const float yaw_rad = yaw_deg * deg2rad;
  const float pitch_rad = pitch_deg * deg2rad;
  float yaw_err_rad = yaw_pos_err * deg2rad;
  float pitch_err_rad = pitch_pos_err * deg2rad;

  const float dead_rad = kPosDeadzoneDeg * deg2rad;
  if (std::fabs(yaw_err_rad) < dead_rad) yaw_err_rad = 0.0f;
  if (std::fabs(pitch_err_rad) < dead_rad) pitch_err_rad = 0.0f;

  // [解析] sp::PID 内部�?set-fdb，所以把 set 写成 fdb+err 来传�?已处理过的误�?�?
  yaw_pos_pid.calc(yaw_rad + yaw_err_rad, yaw_rad);
  pitch_pos_pid.calc(pitch_rad + pitch_err_rad, pitch_rad);

  // [解析] 外环输出就是内环的目标角速度，并且已经被 PID �?max_out 限到 ±kYawMaxRate/±kPitchMaxRate�?
  gimbal_yaw_rate_cmd = yaw_pos_pid.out;
  gimbal_pitch_rate_cmd = pitch_pos_pid.out;


  // ---- 跑飞保护：实测转速远超指令上�?�?断定正反�?失控 ----
  const bool overspeed = (std::fabs(g_yaw_rate_f) > kRateLimitTrip) ||
                         (std::fabs(g_pitch_rate_f) > kRateLimitTrip);
  if (g_runaway_active) {
    if (yaw_stick == 0.0f && pitch_stick == 0.0f) {
      g_runaway_active = false;  // 摇杆回中：解�?
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

  // ---- 速度环：目标角速度 �?实测角速度（分轴极�?+ 死区）→ 电压 ----
  // [解析] 极性翻转作用在"反馈符号"上：某轴反了就把它取负，把正反馈变回负反馈�?
  const float yaw_fb = (kYawRateInvert != kRateLoopInvert) ? -g_yaw_rate_f : g_yaw_rate_f;
  const float pitch_fb = (kPitchRateInvert != kRateLoopInvert) ? -g_pitch_rate_f : g_pitch_rate_f;

  const float yaw_err = gimbal_yaw_rate_cmd - yaw_fb;
  const float pitch_err = gimbal_pitch_rate_cmd - pitch_fb;
  gimbal_yaw_rate_err = yaw_err;
  gimbal_pitch_rate_err = pitch_err;

  // [解析] 死区在这里实现（sp::PID 没有死区参数）：误差小于阈值时把它变成 0�?
  const float yaw_err_dz = (std::fabs(yaw_err) < kYawRateDeadzone) ? 0.0f : yaw_err;
  const float pitch_err_dz = (std::fabs(pitch_err) < kPitchRateDeadzone) ? 0.0f : pitch_err;

  // [解析] 用带"积分分离"的重载：|P 项输出| 大于 kIntegralPause 时不积分，防止大误差时积分顶满�?
  // [解析]   sp::PID 内部�?set-fdb，所以把 set 写成 fdb+err 来实�?误差已被处理�?的效果�?
  yaw_rate_pid.calc(yaw_fb + yaw_err_dz, yaw_fb, kIntegralPause);
  pitch_rate_pid.calc(pitch_fb + pitch_err_dz, pitch_fb, kIntegralPause);

  // ---- 前馈：按目标转速直接给基础电压，不经过增益 → 不影响稳定性 ----
  // [解析] ★★ 这是"增益高了振、低了跟不上"的破局点：
  // [解析]   频率分析证明抖动是"增益相关的极限环"（Kp1.2→35Hz，Kp3.0→52.7Hz），根源是环路延迟。
  // [解析]   靠 P 增益换力度必然牺牲稳定性；前馈按"电压→转速"的经验比直接给力，P 只负责修误差。
  // [解析]   kFF 由实测数据标定：0.85V → 0.5rad/s，即约 1.7 V per (rad/s)。
  const float yaw_ff = kYawRateFF * gimbal_yaw_rate_cmd;
  const float pitch_ff = kPitchRateFF * gimbal_pitch_rate_cmd;

  gimbal_yaw_out_v = sp::limit_min_max(yaw_ff + yaw_rate_pid.out, -kYawMaxVoltage, kYawMaxVoltage);
  gimbal_pitch_out_v =
    sp::limit_min_max(pitch_ff + pitch_rate_pid.out, -kPitchMaxVoltage, kPitchMaxVoltage);

  // [解析] 0V 裸测：把最终电压强行清零（环路照算、记录照记，只是不输出）。
  if (kOutputTestZero) {
    gimbal_yaw_out_v = 0.0f;
    gimbal_pitch_out_v = 0.0f;
  }

  // ---- 飞行记录仪 ----
  {
    // [解析] 0V 裸测时自动触发并循环覆盖：这样任何时刻 dump 出来的都是"最近 512ms"，不用掐时机。
    const bool trigger = kOutputTestZero || (std::fabs(pitch_stick) > 0.15f) ||
                         (std::fabs(yaw_stick) > 0.15f);
    if (g_rec_reset_req) {
      g_rec_reset_req = 0;
      g_rec_state = 0;
      g_rec_n = 0;
    }
    if (g_rec_state == 0 && trigger) {
      g_rec_state = 1;  // 上升沿开始录
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
      else {
        // [解析] 0V 裸测：录满后从头覆盖（保持"最近 512ms"）；正常模式录满即停。
        if (kOutputTestZero) {
          g_rec_n = 0;
          g_rec_tick = 0.0f;
        }
        else {
          g_rec_state = 2;
        }
      }
    }
  }

  gimbal_yaw.cmd(gimbal_yaw_out_v);
  gimbal_pitch.cmd(gimbal_pitch_out_v);
  send_frame();
}

}  // namespace

// [解析] 任务本体�? kHz 固定节拍，函数不会返回（FreeRTOS 不允�?return）�?
extern "C" void gimbal_task(void const * argument)
{
  (void)argument;

  for (;;) {
    gimbal_step();
    osDelay(1);
  }
}

/*==============================================================================
 * 使用与验�?
 *
 * 上电：静�?1s �?之后推杆云台�?摇杆�?× 0.8rad/s"的速度转，松手即刹停�?
 * 注意：这�?*速度�?*，不是位置环 —�?松手后云台停在哪儿就是哪儿，不会回到某个角度�?
 *
 * | 变量                  | 正常�?/ 含义                                  |
 * |-----------------------|------------------------------------------------|
 * | g_gimbal_loop_hz      | �?000                                          |
 * | gimbal_yaw_stick      | 中位 0.000                                     |
 * | gimbal_yaw_rate_cmd   | 推满 ±0.8 rad/s                                |
 * | gimbal_yaw_rate       | 跟随 cmd（滤波后�?                            |
 * | gimbal_yaw_rate_raw   | 未滤波值；�?rate 差很多说明陀螺噪声大          |
 * | gimbal_yaw_out_v      | 通常�?±2V 内；�?±6V 说明跟不动或符号�?       |
 * | g_rate_runaway        | 必须 false                                     |
 * | g_tx_drop_count       | 基本不动                                       |
 *
 * 故障对照�?
 * 1) **推杆后越转越快、停不下�?* �?正反馈，�?kRateLoopInvert 改成 true（或翻转 kYawSign/kPitchSign 之一）�?
 *    跑飞保护会在 200ms 后切断，此时 g_rate_runaway = true，摇杆回中可恢复�?
 * 2) **推杆几乎不动，out_v 很小** �?Kp 太保守，�?kRateKp 1.5 �?3.0；或电机没收到指令（�?CAN/ID/模式）�?
 * 3) **out_v �?±6V（pitch �?±3V）但转不起来** �?电压不够顶静摩擦，把上限调大�?
 * 4) **嗡嗡�?高频�?* �?陀螺噪声被放大：把 kRateKp 降到 1.0，或�?kRateAlpha 提到 0.85（更平滑）�?
 * 5) **只在一个方向抖** �?该轴陀螺符号与另一轴不一致，检�?kRateLoopInvert 是否只该对单轴生效�?
 *
 * 下一步（确认这一版稳了再做）：在前面加位置环（IMU 角度）→ 变成串级，才是完整云台�?
 *============================================================================*/
