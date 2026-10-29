// [解析] Vision 协议层实现：收（USB 中断里切帧） + 发（任务里打包 43 字节） + 掉线判定。
// [解析] 关键约定：
//   1) 帧头 'S''P' 命中但 CRC 错时只丢 1 个字节重新找帧头，而不是整帧丢弃 —— 帧头可能是数据里巧合撞上的；
//   2) 长度不够一帧就原地等下一批数据（USB 全速 CDC 一次回调最多 64 字节，一帧 29 字节不会拆得太碎）；
//   3) 发送用 CDC_Transmit_FS，返回 USBD_BUSY 时直接放弃本帧，绝不在任务里忙等。

#include "io/vision/vision.hpp"

#include <cstring>

#include "main.h"
#include "usbd_cdc_if.h"

namespace
{
// [解析] CRC-16/MCRF4XX：初值 0xFFFF，反射多项式 0x8408（即 0x1021 的镜像），无最终异或。
// [解析] 与视觉侧 tools/crc.cpp 查表实现结果完全一致，这里用位运算省一张 512 字节的表。
uint16_t vision_crc16(const uint8_t * data, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408U) : static_cast<uint16_t>(crc >> 1);
  }
  return crc;
}

// [解析] 协议里的 CRC 是"低字节在前"，所以不能直接比较 uint16 的内存，要按小端拼回来。
inline uint16_t read_le16(const uint8_t * p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

}  // namespace

namespace sp
{

// [解析] 由 usbd_cdc_if.c 的 CDC_Receive_FS 在 USB 中断上下文调用，禁止延时、打印、动态内存。
// [解析] 处理策略：追加 -> 循环切帧 -> 合法帧更新状态 / 非法帧丢 1 字节重对齐。
void Vision::update(uint8_t * buf, uint32_t len)
{
  if (buf == nullptr || len == 0) return;

  // [解析] 单次回调理论上 <= 64 字节；真遇到超长数据只保留尾部，保证不越界。
  if (len > sizeof(rx_buffer_)) {
    buf += (len - sizeof(rx_buffer_));
    len = sizeof(rx_buffer_);
    rx_len_ = 0;
  }

  // [解析] 装不下说明前面有半帧一直没凑齐：整段丢弃重新找帧头，比"丢最老的字节"更容易恢复对齐。
  if (rx_len_ + len > sizeof(rx_buffer_)) {
    rx_len_ = 0;
    rx_resync_bytes++;
  }

  std::memcpy(rx_buffer_ + rx_len_, buf, len);
  rx_len_ = static_cast<uint16_t>(rx_len_ + len);

  while (rx_len_ >= 2) {
    // [解析] 不是帧头：丢 1 字节继续滑窗
    if (rx_buffer_[0] != VISION_HEAD0 || rx_buffer_[1] != VISION_HEAD1) {
      std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_ - 1);
      rx_len_--;
      rx_resync_bytes++;
      continue;
    }

    // [解析] 帧头对了但还没收够一帧：留在缓冲里等下一批数据
    if (rx_len_ < sizeof(VisionToGimbal)) break;

    // [解析] 整帧 CRC 校验：只覆盖末尾 crc16 之前的字节
    const uint16_t crc_calc =
      vision_crc16(rx_buffer_, sizeof(VisionToGimbal) - sizeof(rx_data_.crc16));
    const uint16_t crc_recv = read_le16(rx_buffer_ + sizeof(VisionToGimbal) - 2);

    if (crc_calc != crc_recv) {
      // [解析] CRC 错：帧头可能是巧合，丢 1 字节重找，不要整帧跳 29 字节
      std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_ - 1);
      rx_len_--;
      rx_crc_error++;
      continue;
    }

    std::memcpy(&rx_data_, rx_buffer_, sizeof(rx_data_));

    // [解析] 解析成对控制友好的形式：mode 0 不接管，2 才允许开火
    control_ = (rx_data_.mode != 0);
    fire_ = (rx_data_.mode == 2);
    yaw_ = rx_data_.yaw;
    yaw_vel_ = rx_data_.yaw_vel;
    yaw_acc_ = rx_data_.yaw_acc;
    pitch_ = rx_data_.pitch;
    pitch_vel_ = rx_data_.pitch_vel;
    pitch_acc_ = rx_data_.pitch_acc;

    last_rx_ms_ = HAL_GetTick();
    has_rx_ = true;
    rx_frame_count++;

    rx_len_ = static_cast<uint16_t>(rx_len_ - sizeof(VisionToGimbal));
    if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + sizeof(VisionToGimbal), rx_len_);
  }
}

// [解析] 任务里周期调用（建议 1 kHz），把本车状态打包成 43 字节发出。
// [解析] 只由 vision_task 一个线程调用，所以 tx_data_ 不需要加锁。
uint8_t Vision::send(
  uint8_t mode, const float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
  float bullet_speed, uint16_t bullet_count)
{
  tx_data_.mode = mode;
  tx_data_.q[0] = q[0];
  tx_data_.q[1] = q[1];
  tx_data_.q[2] = q[2];
  tx_data_.q[3] = q[3];
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.bullet_speed = bullet_speed;
  tx_data_.bullet_count = bullet_count;

  // [解析] 帧头不用赋值：结构体默认成员初始化器已经是 {'S','P'}
  tx_data_.crc16 = vision_crc16(
    reinterpret_cast<const uint8_t *>(&tx_data_),
    sizeof(tx_data_) - sizeof(tx_data_.crc16));

  const uint8_t ret =
    CDC_Transmit_FS(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));

  // [解析] USB 上一帧还没发完（主机没来取）时返回 BUSY，本帧直接丢，下一毫秒再来
  if (ret == USBD_BUSY) tx_drop_count++;
  return ret;
}

// [解析] 取一帧完整快照：关中断期间拷贝，保证 yaw/yaw_vel/yaw_acc 等来自同一帧。
bool Vision::get_control(Control & out)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  out.control = control_;
  out.fire = fire_;
  out.yaw = yaw_;
  out.yaw_vel = yaw_vel_;
  out.yaw_acc = yaw_acc_;
  out.pitch = pitch_;
  out.pitch_vel = pitch_vel_;
  out.pitch_acc = pitch_acc_;
  out.stamp_ms = last_rx_ms_;

  if (!primask) __enable_irq();

  // [解析] 掉线判定放在临界区外：HAL_GetTick 自己会读 SysTick 计数
  out.valid = has_rx_ && out.control && alive(HAL_GetTick());
  return out.valid;
}

// [解析] 视觉在线判定：必须收到过帧，且距最后一帧 < 100 ms
bool Vision::alive(uint32_t now_ms) const
{
  return has_rx_ && (now_ms - last_rx_ms_) < VISION_ALIVE_TIMEOUT_MS;
}

}  // namespace sp
