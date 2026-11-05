#include "vision_task.hpp"

#include <cmath>
#include <cstring>

#include "cmsis_os.h"
#include "tools/crc/crc.hpp"
#include "usbd_cdc_if.h"
#include "vision_link.h"

/**
 * @file vision_task.cpp
 * @brief USB virtual COM port <-> firmware bridge.
 *
 * Receive: accumulates raw bytes from the CDC ring buffer, scans for the "SP"
 * header, and only accepts a 29 byte frame whose CRC16 matches. Any byte that
 * cannot start a valid frame is discarded, so the parser re-synchronises
 * without needing to know where frame boundaries are.
 *
 * Transmit: emits a 43 byte telemetry frame every TELEMETRY_PERIOD_MS. The
 * transmit path is non-blocking, so a frame is simply skipped if the previous
 * one has not drained yet - telemetry is periodic and a dropped sample is
 * harmless, whereas blocking would stall command parsing.
 */

namespace
{

constexpr std::size_t RX_BUFFER_SIZE = 2 * vision::VISION_TO_GIMBAL_SIZE;

uint8_t rx_buffer_[RX_BUFFER_SIZE];
std::size_t rx_count_ = 0;

vision::GimbalFeedback feedback_;
vision::CommandState command_;

uint32_t command_frame_count_ = 0;
uint32_t command_error_count_ = 0;

/// Tick of the last accepted command, used for the liveness timeout.
uint32_t last_command_tick_ = 0;

float clampf(float value, float low, float high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

/// Accept the command only if every field is usable. A NaN would otherwise
/// propagate straight into whatever consumes the angle.
bool fields_are_finite(const vision::VisionToGimbal & frame)
{
  return std::isfinite(frame.yaw) && std::isfinite(frame.yaw_vel) && std::isfinite(frame.yaw_acc) &&
         std::isfinite(frame.pitch) && std::isfinite(frame.pitch_vel) &&
         std::isfinite(frame.pitch_acc);
}

void mark_control_lost()
{
  command_.control = false;
  command_.fire = false;
  command_.yaw = 0.0f;
  command_.yaw_vel = 0.0f;
  command_.yaw_acc = 0.0f;
  command_.pitch = 0.0f;
  command_.pitch_vel = 0.0f;
  command_.pitch_acc = 0.0f;

  if (feedback_.mode == vision::GIMBAL_AUTO_AIM) {
    feedback_.mode = vision::GIMBAL_IDLE;
  }
}

/// Turn one validated frame into command state.
void apply_command(const vision::VisionToGimbal & frame)
{
  const bool wants_control = (frame.mode != vision::MODE_NO_CONTROL);

  command_.control = wants_control;
  command_.fire = (frame.mode == vision::MODE_CONTROL_AND_FIRE);
  command_.alive = true;

  command_.yaw = clampf(frame.yaw, -static_cast<float>(M_PI), static_cast<float>(M_PI));
  command_.pitch = clampf(frame.pitch, -static_cast<float>(M_PI), static_cast<float>(M_PI));

  // Feed-forward terms are bounded to keep a corrupt-but-CRC-valid frame from
  // injecting an absurd acceleration.
  command_.yaw_vel = clampf(frame.yaw_vel, -50.0f, 50.0f);
  command_.pitch_vel = clampf(frame.pitch_vel, -50.0f, 50.0f);
  command_.yaw_acc = clampf(frame.yaw_acc, -500.0f, 500.0f);
  command_.pitch_acc = clampf(frame.pitch_acc, -500.0f, 500.0f);

  last_command_tick_ = osKernelSysTick();
  command_frame_count_++;

  // The upper computer treats the telemetry mode as "what is this robot doing",
  // so mirror the request back once a command is live.
  feedback_.mode = wants_control ? vision::GIMBAL_AUTO_AIM : vision::GIMBAL_IDLE;
}

/// Try to extract as many frames as possible from the accumulation buffer.
void parse_incoming()
{
  std::size_t consumed = 0;

  while (rx_count_ - consumed >= vision::VISION_TO_GIMBAL_SIZE) {
    const uint8_t * candidate = rx_buffer_ + consumed;

    if (candidate[0] != vision::HEAD_0 || candidate[1] != vision::HEAD_1) {
      // Not a frame start; drop this single byte. Dropped bytes are not
      // counted as errors because during normal operation the buffer usually
      // ends mid-header, and counting those would drown the real error rate.
      consumed++;
      continue;
    }

    if (!sp::check_crc16(candidate, static_cast<uint32_t>(vision::VISION_TO_GIMBAL_SIZE))) {
      // Header looks right but the payload does not checksum, so the stream is
      // misaligned: advance one byte and keep looking for a genuine header.
      consumed++;
      command_error_count_++;
      continue;
    }

    vision::VisionToGimbal frame;
    std::memcpy(&frame, candidate, sizeof(frame));

    if (fields_are_finite(frame)) {
      apply_command(frame);
    } else {
      command_error_count_++;
    }

    consumed += vision::VISION_TO_GIMBAL_SIZE;
  }

  // Keep the trailing bytes that may hold the beginning of a frame: at most
  // VISION_TO_GIMBAL_SIZE - 1 of them can matter.
  const std::size_t keep_limit = vision::VISION_TO_GIMBAL_SIZE - 1;
  if (rx_count_ - consumed > keep_limit) {
    consumed = rx_count_ - keep_limit;
  }

  if (consumed > 0) {
    rx_count_ -= consumed;
    if (rx_count_ > 0) {
      std::memmove(rx_buffer_, rx_buffer_ + consumed, rx_count_);
    }
  }
}

/// Drain the CDC ring buffer into the accumulation buffer.
void poll_usb()
{
  if (!CDC_IsConnected()) {
    // No host: discard any partial frame so a stale prefix cannot be glued to
    // the first frame of the next session.
    rx_count_ = 0;
    CDC_RxFlush();
    return;
  }

  while (rx_count_ < RX_BUFFER_SIZE) {
    const uint16_t room = static_cast<uint16_t>(RX_BUFFER_SIZE - rx_count_);
    const uint16_t got = CDC_RxRead(rx_buffer_ + rx_count_, room);
    if (got == 0) {
      break;
    }
    rx_count_ += got;
  }

  // The buffer only fills up if a complete frame is somehow never recognised;
  // reset rather than staying permanently deaf.
  if (rx_count_ >= RX_BUFFER_SIZE) {
    rx_count_ = 0;
    command_error_count_++;
  }
}

/// Build and queue one telemetry frame.
void send_telemetry()
{
  static vision::GimbalToVision frame;

  // Rebuilding the header each time keeps the struct usable even if it is
  // stored in .bss and the application never touched `head`.
  frame.head[0] = vision::HEAD_0;
  frame.head[1] = vision::HEAD_1;
  frame.mode = static_cast<uint8_t>(feedback_.mode);

  frame.q[0] = feedback_.q[0];
  frame.q[1] = feedback_.q[1];
  frame.q[2] = feedback_.q[2];
  frame.q[3] = feedback_.q[3];

  frame.yaw = feedback_.yaw;
  frame.yaw_vel = feedback_.yaw_vel;
  frame.pitch = feedback_.pitch;
  frame.pitch_vel = feedback_.pitch_vel;
  frame.bullet_speed = feedback_.bullet_speed;
  frame.bullet_count = feedback_.bullet_count;

  frame.crc16 = sp::get_crc16(
    reinterpret_cast<const uint8_t *>(&frame),
    static_cast<uint32_t>(sizeof(frame) - sizeof(frame.crc16)));

  // Non-blocking: skip this period if the previous frame is still in flight.
  (void)CDC_Transmit_FS(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
}

}  // namespace

namespace vision
{

uint32_t command_frame_count() { return command_frame_count_; }

uint32_t command_error_count() { return command_error_count_; }

bool usb_connected() { return CDC_IsConnected() != 0U; }

CommandState command_state() { return command_; }

GimbalFeedback & feedback() { return feedback_; }

}  // namespace vision

extern "C" void vision_task(void const * argument)
{
  (void)argument;

  uint32_t last_telemetry_tick = osKernelSysTick();
  last_command_tick_ = osKernelSysTick();

  for (;;) {
    poll_usb();
    parse_incoming();

    // Liveness: losing the upper computer must drop control, otherwise the
    // last command would be held indefinitely.
    if (command_.control &&
        (osKernelSysTick() - last_command_tick_) > vision::COMMAND_TIMEOUT_MS) {
      command_.alive = false;
      mark_control_lost();
    }

    if ((osKernelSysTick() - last_telemetry_tick) >= vision::TELEMETRY_PERIOD_MS) {
      last_telemetry_tick = osKernelSysTick();
      if (CDC_IsConnected()) {
        send_telemetry();
      }
    }

    // 1 ms keeps the receive path responsive without starving other tasks.
    osDelay(1);
  }
}
