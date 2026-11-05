#ifndef APPLICATIONS__VISION__VISION_TASK_HPP
#define APPLICATIONS__VISION__VISION_TASK_HPP

#include <cstdint>

#include "protocol.hpp"

/**
 * @file vision_task.hpp
 * @brief Bridges the USB virtual COM port with the rest of the firmware.
 *
 * The task parses `vision::VisionToGimbal` command frames from the upper
 * computer and periodically emits a `vision::GimbalToVision` telemetry frame.
 *
 * The task entry point itself is declared in `vision_link.h`, which is C-safe
 * because `freertos.c` is compiled as C.
 */

namespace vision
{

/// Telemetry period in milliseconds. 100 Hz is well within USB full-speed
/// budget (43 bytes/frame) and matches the upper computer's expectations.
constexpr uint32_t TELEMETRY_PERIOD_MS = 10;

/// If no valid command arrives within this window the link is considered dead
/// and the command fields are zeroed.
constexpr uint32_t COMMAND_TIMEOUT_MS = 100;

/**
 * @brief Snapshot of the state reported back to the upper computer.
 *
 * Filled in by whoever owns the gimbal. Until a real gimbal exists these stay
 * at their initial values, but the upper computer still needs a correctly
 * framed telemetry stream to interpolate gimbal attitude against image
 * timestamps.
 */
struct GimbalFeedback
{
  /// Gimbal-to-world rotation as a quaternion in wxyz order.
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  float yaw = 0.0f;        ///< rad
  float yaw_vel = 0.0f;    ///< rad/s
  float pitch = 0.0f;      ///< rad
  float pitch_vel = 0.0f;  ///< rad/s

  float bullet_speed = 23.0f;  ///< m/s, nominal for a 42 mm 15 m/s muzzle
  uint16_t bullet_count = 0;   ///< cumulative shots

  GimbalMode mode = GIMBAL_IDLE;
};

/// Latest command accepted from the upper computer.
struct CommandState
{
  bool control = false;  ///< gimbal control is being requested
  bool fire = false;     ///< firing is being requested
  bool alive = false;    ///< a valid frame arrived within COMMAND_TIMEOUT_MS

  float yaw = 0.0f;        ///< rad, absolute target angle
  float yaw_vel = 0.0f;    ///< rad/s feed-forward
  float yaw_acc = 0.0f;    ///< rad/s^2 feed-forward
  float pitch = 0.0f;      ///< rad
  float pitch_vel = 0.0f;  ///< rad/s
  float pitch_acc = 0.0f;  ///< rad/s^2
};

/// Number of command frames accepted since boot (diagnostics).
uint32_t command_frame_count();

/// Number of frames discarded because of a CRC or header error.
uint32_t command_error_count();

/// Whether the USB port is configured by a host.
bool usb_connected();

/// Copy the latest accepted command.
CommandState command_state();

/// Read/modify the telemetry payload that is sent on the next period.
GimbalFeedback & feedback();

}  // namespace vision

#endif  // APPLICATIONS__VISION__VISION_TASK_HPP
