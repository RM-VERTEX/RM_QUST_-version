// [解析] 底盘控制链路（四全向轮 omni / 步兵底盘），本期最轻量实现：
// [解析]   DBus 遥控器 -> (vx, vy, wz) -> sp::Omni 运动学 -> 四路速度环 sp::PID
// [解析]   -> CAN1 标准帧 0x200（一帧装 4 台 M3508 的电流）-> 电机
// [解析] 一个任务独占 CAN1：发送在本任务、接收在文件末尾的 FIFO0 中断回调，避免多任务抢发送邮箱。
// [解析] 周期是 osDelay(1)≈1ms，PID 的 dt 写死 1e-3f，两者必须一致；改周期要连 dt 一起改。
// [解析] 超级电容与功率控制硬件尚未就绪，接口预留在本文件下半部分的"预留接口"区。

#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/omni/omni.hpp"
#include "tools/pid/pid.hpp"

// [解析] remote 的对象定义在 uart_task.cpp（DBus 接收任务），本任务只读它解析好的结果，不再加锁。
extern sp::DBus remote;

// [解析] 云台的 2 台 GM6020 定义在 gimbal_task.cpp；那个任务独占控制，本任务只负责把 0x205/0x206 帧转交过去。
extern sp::RM_Motor gimbal_yaw;
extern sp::RM_Motor gimbal_pitch;

namespace
{
/*==================== 机械参数（必须按实车实测修改）====================*/
// [解析] 这三个数决定运动学是否正确，量错会让底盘"斜着走"或走不直，务必用卷尺实测。
constexpr float kWheelRadius = 0.076f;  // 轮子半径, 单位: m（4 寸全向轮, 直径 152mm）
constexpr float kHalfLength = 0.150f;   // 底盘中心到前/后轮轴线的距离, 单位: m
constexpr float kHalfWidth = 0.150f;  // 底盘中心到左/右轮轴线的距离, 单位: m（正方形底盘两者相等）

/*==================== 电机 ID 与装配方向 ====================*/
// [解析] ID 顺序沿用 sp_middleware/tools/omni/readme.md「用法」示例的约定, 绕底盘逆时针一圈:
// [解析]         ID1 = 右前(RF)   ID2 = 左前(LF)   ID3 = 左后(LR)   ID4 = 右后(RR)
// [解析] ID 必须和 C620 电调拨码一致; 四台必须同属 1~4（一条 0x200 帧只装得下 4 台）。
// [解析] 某个轮子转向反了, 就把对应的 kReverseXx 改成 true, 不要去改 omni 的公式符号
// [解析]   （标定方法见 sp_middleware/tools/omni/readme.md 第"reverse_xx"节）。
constexpr uint8_t kIdRf = 1;  // 右前
constexpr uint8_t kIdLf = 2;  // 左前
constexpr uint8_t kIdLr = 3;  // 左后
constexpr uint8_t kIdRr = 4;  // 右后

constexpr bool kReverseRf = false;
constexpr bool kReverseLf = false;
constexpr bool kReverseLr = false;
constexpr bool kReverseRr = false;

// [解析] 减速箱 268:17 ≈ 15.76（原装 M3508 P19 是 3591:187 ≈ 19.2）。比原装少 18% 减速, 后果:
// [解析]   同一个转子转速下, 轮上扭矩 ×0.82、轮速 ×1.22。反馈与指令都由这个值换算到轮端。
constexpr float kRatio = 268.0f / 17.0f;

/*==================== 控制参数 ====================*/
constexpr float kDt = 1e-3f;             // 控制周期, 单位: s, 必须与 osDelay(1) 一致
constexpr float kMaxSpeed = 1.5f;        // 最大平移速度, 单位: m/s（首次上电建议先降到 0.5）
constexpr float kMaxOmega = 2.0f;        // 最大自转角速度, 单位: rad/s
constexpr float kStickDeadzone = 0.05f;  // 摇杆死区, 防中位抖动导致底盘缓慢爬行

// [解析] 速度环的输出量纲是"输出轴力矩 N·m"。厂商手册(《M3508 使用说明 V1.0》特征参数)是 P19 口径:
// [解析]   转矩常数 0.3 N·m/A, 额定(最大连续)转矩 3 N·m @ 10A, 堵转扭矩 4.5 N·m。
// [解析] 换成 268:17 后, 输出轴常数按减速比折算 = 0.3 × (268/17)/(3591/187) = 0.246 N·m/A:
// [解析]   1 N·m ≈ 4.06A；满量程(16384 raw = 20A) ≈ 4.93 N·m；额定连续 ≈ 2.46 N·m。
// [解析]   所以 kMaxTorque = 1.2 N·m ≈ 4.9A, 约为额定连续的 49%, 热上仍有余量。
// [解析] 下面这组参数是保守的起步值, 上电后按注释里的顺序调, 不要一次改多个。
// [解析] kp 的取值由 tools/verify_chassis_sim.cpp 的离线仿真给出: 0.30→0.60 把阶跃调节时间
// [解析]   从 1.64 倍物理下限压到 1.20 倍, 再往上加收益很小; 全程超调 < 0.1%。
constexpr float kKp = 0.60f;  // 比例, 单位: N·m/(rad/s)。先加到"能跟上、不啸叫"
constexpr float kKi = 0.40f;  // 积分, 用来消静差。平台期还达不到目标速度就加大
constexpr float kKd = 0.0f;   // 速度环一般不用微分
// [解析] 这一项才是"跟不跟手"的真正瓶颈: 1.2 N·m ≈ 3.2 m/s² 整车加速度, 2.4 N·m ≈ 6.3, 3.6 ≈ 9.5。
// [解析]   想要更冲就先加它（同时盯住电流/电池), 而不是去加 kp。
constexpr float kMaxTorque = 1.2f;  // 单轮力矩上限, 单位: N·m（≈4.9A）。急加速抖动就往下调
constexpr float kMaxIout = 0.60f;   // 积分限幅, 单位: N·m。抗积分饱和, 至少要能兜住静摩擦
// [解析] 积分分离阈值: |比例项| 超过它（即误差较大）时本拍不积分, 避免起步阶段把 iout 堆满后超调。
constexpr float kIntegralPause = 0.6f;  // 单位: N·m, 约等于 2 rad/s 的误差

/*==================== 对象 ====================*/
sp::CAN can1(&hcan1);

// [解析] ratio 传减速比, 之后 motor.speed 就是轮端 rad/s（正好是 omni.calc() 的单位）,
// [解析]   motor.cmd() 的入参就是输出轴 N·m, 中间不需要再手工换算了。
// [解析] 声明顺序 = ID 顺序（ID1 右前 → ID4 右后），这样和拨码表能一眼对上。
sp::RM_Motor motor_rf(kIdRf, sp::RM_Motors::M3508, kRatio);
sp::RM_Motor motor_lf(kIdLf, sp::RM_Motors::M3508, kRatio);
sp::RM_Motor motor_lr(kIdLr, sp::RM_Motors::M3508, kRatio);
sp::RM_Motor motor_rr(kIdRr, sp::RM_Motors::M3508, kRatio);

// [解析] 解算器只认"轮子位于四角、滚动方向垂直于对角线", l == w 时就是常见的 45° X 型布局。
// [解析] 注意 Omni 构造参数的顺序是 (lf, lr, rf, rr), 不是 (lf, rf, lr, rr) —— 写反会让
// [解析]   "翻右后"实际翻了左后, 而且四个都为 false 时完全看不出来。
sp::Omni chassis(
  kWheelRadius, kHalfLength, kHalfWidth, kReverseLf, kReverseLr, kReverseRf, kReverseRr);

// [解析] 四路速度环参数完全一致, 只是设定值与反馈值不同, 所以用同一个构造函数反复建对象。
sp::PID pid_rf(kDt, kKp, kKi, kKd, kMaxTorque, kMaxIout);
sp::PID pid_lf(kDt, kKp, kKi, kKd, kMaxTorque, kMaxIout);
sp::PID pid_lr(kDt, kKp, kKi, kKd, kMaxTorque, kMaxIout);
sp::PID pid_rr(kDt, kKp, kKi, kKd, kMaxTorque, kMaxIout);

/*==========================================================================
 *                        预留接口：功率控制 / 超级电容
 * 当前硬件未就绪，这里只做两件事：估算底盘机械功率（供调试观察）+
 * 用固定总力矩上限兜底，防止没有功率控制时一把拉爆电池。
 * 接入顺序见文件末尾的说明。
 *========================================================================*/
constexpr float kMaxTotalTorque = 3.6f;  // 四轮力矩绝对值之和的上限 ≈14.6A 总线电流 ≈350W@24V

// [解析] 机械功率 = Σ|转速 × 力矩|; 电气功率约为它的 1.3~1.5 倍（铜损 + 铁损）。
// [解析] 将来真正的功率环应该拿这个值去和裁判系统给的功率上限比较，而不是靠常数兜底。
float g_chassis_power_w = 0.0f;

// [解析] 这里是功率控制的唯一接入点：硬件就绪后把"固定上限"换成"功率环算出的上限"即可。
// [解析] 具体两步：① 用裁判系统 0x0202 的缓冲能量算功率上限；② 把上限写进超电模块。
void chassis_power_update(float power_w)
{
  g_chassis_power_w = power_w;

  // 预留①：超电下发（sp_middleware/motor/super_cap/super_cap.cpp 加入编译后启用）
  // super_cap.write(can1.tx_data, power_limit, buffer_energy, 1);
  // can1.send(super_cap.tx_id);
}

// [解析] 只按"总力矩"限幅，不做功率闭环；四个轮子等比例缩小, 保持合成方向不失真。
void chassis_apply_power_limit(float torque[4])
{
  float sum = 0.0f;
  for (int i = 0; i < 4; i++) sum += std::fabs(torque[i]);

  if (sum > kMaxTotalTorque) {
    const float scale = kMaxTotalTorque / sum;
    for (int i = 0; i < 4; i++) torque[i] *= scale;
  }
}

/*==================== 内部函数 ====================*/
// [解析] 摇杆中位有几百个计数的抖动（归一化后约 0.01~0.03），不加死区底盘会一直缓慢爬行。
float deadzone(float value) { return (std::fabs(value) < kStickDeadzone) ? 0.0f : value; }

// [解析] 一帧 0x200 装四台电机的电流：先让四台各写自己那 2 字节，再发一次。
void send_current_frame()
{
  motor_lf.write(can1.tx_data);
  motor_rf.write(can1.tx_data);
  motor_lr.write(can1.tx_data);
  motor_rr.write(can1.tx_data);

  // [解析] 邮箱满时 HAL 只返回错误不会报出来，先判空再发，避免静默丢帧（丢帧电机保持上一拍电流）。
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) can1.send(motor_lf.tx_id);
}

// [解析] 失联/异常时的统一出口：清掉积分（否则积分残留会让下一次恢复瞬间猛冲）并下发 0 电流。
void stop_chassis()
{
  pid_lf.clear();
  pid_rf.clear();
  pid_lr.clear();
  pid_rr.clear();

  motor_lf.cmd(0.0f);
  motor_rf.cmd(0.0f);
  motor_lr.cmd(0.0f);
  motor_rr.cmd(0.0f);

  send_current_frame();
}

// [解析] 一个控制周期的完整链路。它是纯函数式的一趟：读输入 -> 算 -> 下发，不保存跨周期状态。
void chassis_step()
{
  const uint32_t now_ms = osKernelSysTick();

  // ---- 1. 仲裁：遥控器或任意一台电机不在线，就整车置零（不做"半残"控制）----
  const bool rc_ok = remote.is_alive(now_ms);
  const bool motors_ok = motor_lf.is_alive(now_ms) && motor_rf.is_alive(now_ms) &&
                         motor_lr.is_alive(now_ms) && motor_rr.is_alive(now_ms);
  if (!rc_ok || !motors_ok) {
    stop_chassis();
    return;
  }

  // ---- 2. 遥控器 -> 底盘速度指令 ----
  // [解析] 坐标系约定(x 向前, y 向左, 逆时针为正):
  // [解析]   右摇杆上下 -> vx; 右摇杆左右 -> vy(右推要往右走, 故取负); 左摇杆左右 -> wz(右推顺时针, 故取负)。
  const float vx = sp::limit_min_max(deadzone(remote.ch_rv), -1.0f, 1.0f) * kMaxSpeed;
  const float vy = -sp::limit_min_max(deadzone(remote.ch_rh), -1.0f, 1.0f) * kMaxSpeed;
  const float wz = -sp::limit_min_max(deadzone(remote.ch_lh), -1.0f, 1.0f) * kMaxOmega;

  // ---- 3. 运动学：整车速度 -> 四个轮子的目标转速(rad/s) ----
  chassis.calc(vx, vy, wz);

  // ---- 4. 四路速度环：反馈用电机反馈的轮端角速度, 输出是力矩 ----
  // [解析] 用带积分分离的重载: 误差大时只走比例项, 误差小了才开始积分, 起步不会先堆满积分再冲过头。
  // [解析] 下面四行与对象声明同序（ID1 右前 → ID4 右后），torque[] 的下标也按这个顺序。
  pid_rf.calc(chassis.speed_rf, motor_rf.speed, kIntegralPause);
  pid_lf.calc(chassis.speed_lf, motor_lf.speed, kIntegralPause);
  pid_lr.calc(chassis.speed_lr, motor_lr.speed, kIntegralPause);
  pid_rr.calc(chassis.speed_rr, motor_rr.speed, kIntegralPause);

  float torque[4] = {pid_rf.out, pid_lf.out, pid_lr.out, pid_rr.out};

  // ---- 5. 功率/电流上限（预留接口）----
  // [解析] 机械功率 = Σ|实测转速 × 力矩|, 用实测值而不是目标值, 将来才和裁判系统的功率对得上。
  const float power_w =
    std::fabs(motor_rf.speed * torque[0]) + std::fabs(motor_lf.speed * torque[1]) +
    std::fabs(motor_lr.speed * torque[2]) + std::fabs(motor_rr.speed * torque[3]);
  chassis_power_update(power_w);
  chassis_apply_power_limit(torque);

  // ---- 6. 下发 ----
  motor_rf.cmd(torque[0]);
  motor_lf.cmd(torque[1]);
  motor_lr.cmd(torque[2]);
  motor_rr.cmd(torque[3]);
  send_current_frame();
}

}  // namespace

// [解析] 任务本体：先配置 CAN 过滤器与中断，然后进入 1ms 的固定节拍。函数不会返回（FreeRTOS 不允许 return）。
extern "C" void chassis_task(void const * argument)
{
  (void)argument;

  can1.config();  // 全通配过滤器（调试阶段先全收），真正按 ID 分发在下面的接收回调里
  can1.start();

  for (;;) {
    chassis_step();
    osDelay(1);
  }
}

// [解析] CAN1 接收回调：四个 M3508 的反馈都从这里进（0x201~0x204），解析后写进各自的 motor.speed/angle。
// [解析] 必须"先把 FIFO 取空再判 ID"，反过来写会在总线上出现无关帧时卡死在中断里。
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;

  const uint32_t stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    can1.recv();

    if (can1.rx_id == motor_rf.rx_id)
      motor_rf.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motor_lf.rx_id)
      motor_lf.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motor_lr.rx_id)
      motor_lr.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motor_rr.rx_id)
      motor_rr.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == gimbal_yaw.rx_id)
      gimbal_yaw.read(can1.rx_data, stamp_ms);  // 云台 Yaw  (GM6020 ID1, 0x205)
    else if (can1.rx_id == gimbal_pitch.rx_id)
      gimbal_pitch.read(can1.rx_data, stamp_ms);  // 云台 Pitch(GM6020 ID2, 0x206)
    // 其它 ID（超电 0x301/0x302 等）在这里按需接进来，当前直接丢弃
  }
}

/*==============================================================================
 * 预留接口接线说明（硬件到货后按这个顺序改，不需要动上面的控制链路）
 *
 * 1) 超级电容
 *    - CMakeLists.txt 加入 sp_middleware/motor/super_cap/super_cap.cpp
 *    - 在本文件建对象：   sp::SuperCap super_cap;
 *    - 接收：在 HAL_CAN_RxFifo0MsgPendingCallback 里补
 *              else if (can1.rx_id == super_cap.super_cap_rx_id ||
 *                       can1.rx_id == super_cap.buffer_cap_rx_id)
 *                super_cap.read(can1.rx_data, stamp_ms);
 *    - 发送：在 chassis_power_update() 里填 super_cap.write(...) + can1.send(super_cap.tx_id)
 *            注意超电反馈是小端、和电机的帧格式相反。
 *
 * 2) 功率上限（裁判系统）
 *    - CMakeLists.txt 加入 sp_middleware/referee/vt03/vt03.cpp（或 pm02）
 *    - 由 0x0202 拿"底盘功率上限 + 缓冲能量"，算出本拍的功率上限，
 *      再把 chassis_apply_power_limit() 里的 kMaxTotalTorque 换成由它换算出的力矩上限。
 *
 * 3) 闭环节拍
 *    - 若功率环带宽要求高于 1kHz，不要在本任务里加循环，改成独立任务 + 独立 CAN 帧即可。
 *============================================================================*/
