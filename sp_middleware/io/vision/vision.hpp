// [解析] C 板(电控) <-> 视觉上位机 的 USB CDC 虚拟串口协议层。
// [解析] 字节流格式固定：帧头 'S''P' + 定长数据 + CRC16(小端)，两端结构体必须逐字节一致。
// [解析] 本文件与视觉仓库 sp_vision_25/io/gimbal/gimbal.hpp 严格对齐：上行 43 B、下行 29 B。
// [解析] 收发分工：update() 在 USB 接收中断里跑（只解析，不许延时/打印），send() 在任务里周期调用。
// [解析] CRC 是 CRC-16/MCRF4XX：初值 0xFFFF、反射多项式 0x8408、结果低字节在前，覆盖"末尾 2 字节之前"的全部字节。

#ifndef SP__VISION_HPP
#define SP__VISION_HPP

#include <cstddef>
#include <cstdint>

namespace sp
{
// [解析] 帧头与掉线阈值（两次收到合法帧间隔超过 100 ms 就认为视觉掉线）。
constexpr uint8_t VISION_HEAD0 = 'S';
constexpr uint8_t VISION_HEAD1 = 'P';
constexpr uint32_t VISION_ALIVE_TIMEOUT_MS = 100;

// ============================================================================
// 板 -> 机：本车状态，43 字节
// [解析] 顺序、类型、长度都不能动，视觉侧按同样的内存布局直接 memcpy。
// ============================================================================
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {VISION_HEAD0, VISION_HEAD1};
  uint8_t mode;         // 0 空闲, 1 自瞄, 2 小符, 3 大符（操作手档位，由板子决定）
  float q[4];           // 姿态四元数，wxyz 顺序
  float yaw;            // rad
  float yaw_vel;        // rad/s
  float pitch;          // rad
  float pitch_vel;      // rad/s
  float bullet_speed;   // m/s
  uint16_t bullet_count;  // 累计发弹数，视觉靠它判断"这一枪打出去了"
  uint16_t crc16;       // 覆盖前面 41 字节
};
static_assert(sizeof(GimbalToVision) == 43, "GimbalToVision must be 43 bytes");
static_assert(
  offsetof(GimbalToVision, crc16) == sizeof(GimbalToVision) - 2, "crc16 must be the last field");

// ============================================================================
// 机 -> 板：云台控制指令，29 字节
// [解析] mode: 0 不控制（云台保持原位/交回电控）, 1 控云台不开火, 2 控云台且开火。
// [解析] 这里给的是"期望角度 + 角速度 + 角加速度"，是前馈参考量，闭环仍由电控自己做。
// ============================================================================
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {VISION_HEAD0, VISION_HEAD1};
  uint8_t mode;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;  // 覆盖前面 27 字节
};
static_assert(sizeof(VisionToGimbal) == 29, "VisionToGimbal must be 29 bytes");
static_assert(
  offsetof(VisionToGimbal, crc16) == sizeof(VisionToGimbal) - 2, "crc16 must be the last field");

// [解析] 协议层对象：一个工程只建一个，收在 USB 中断、发在 vision_task。
class Vision
{
public:
  // [解析] 给云台控制任务用的"一帧指令快照"，避免读到半新半旧的数据。
  struct Control
  {
    bool valid;    // 视觉在线且 mode != 0 才为 true
    bool control;  // mode != 0
    bool fire;     // mode == 2
    float yaw, yaw_vel, yaw_acc;
    float pitch, pitch_vel, pitch_acc;
    uint32_t stamp_ms;  // 该帧收到的时刻（HAL_GetTick）
  };

  // [解析] USB 接收中断里调用：把这一批字节追加进缓冲，逐帧校验后更新内部状态。
  void update(uint8_t * buf, uint32_t len);

  // [解析] 任务里周期调用：把本车状态打包成 43 字节发出。
  // [解析] 返回 CDC 的原始结果（0=OK、1=BUSY…），USBD_BUSY 时本帧被丢弃并计入 tx_drop_count。
  uint8_t send(
    uint8_t mode, const float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
    float bullet_speed, uint16_t bullet_count);

  // [解析] 取最新指令快照：内部关中断拷贝，返回后调用者拿到的 9 个量一定来自同一帧。
  bool get_control(Control & out);

  // [解析] 视觉是否在线：收到过帧，且距最后一帧 < 100 ms。
  bool alive(uint32_t now_ms) const;

  // [解析] 调试计数，用调试器(f5) 看即可，不参与控制。
  volatile uint32_t rx_frame_count = 0;   // 收到的合法帧数
  volatile uint32_t rx_crc_error = 0;     // 帧头对了但 CRC 错的次数
  volatile uint32_t rx_resync_bytes = 0;  // 为了重新对齐帧头而丢弃的字节数
  volatile uint32_t tx_drop_count = 0;    // USB 忙导致没发出去的帧数

private:
  VisionToGimbal rx_data_{};
  GimbalToVision tx_data_{};

  uint8_t rx_buffer_[128]{};  // 29 字节一帧，128 够放 4 帧；解析及时就不会溢出
  uint16_t rx_len_ = 0;

  volatile bool has_rx_ = false;
  volatile uint32_t last_rx_ms_ = 0;

  volatile bool control_ = false;
  volatile bool fire_ = false;
  volatile float yaw_ = 0.0f;
  volatile float yaw_vel_ = 0.0f;
  volatile float yaw_acc_ = 0.0f;
  volatile float pitch_ = 0.0f;
  volatile float pitch_vel_ = 0.0f;
  volatile float pitch_acc_ = 0.0f;
};

}  // namespace sp

#endif  // SP__VISION_HPP
