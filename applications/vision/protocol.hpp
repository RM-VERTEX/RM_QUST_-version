#ifndef APPLICATIONS__VISION__PROTOCOL_HPP
#define APPLICATIONS__VISION__PROTOCOL_HPP

#include <cstddef>
#include <cstdint>

/**
 * @file protocol.hpp
 * @brief On-wire protocol of the USB virtual COM port between this board and
 *        the upper computer running sp_vision.
 *
 * IMPORTANT: these two structures must stay byte-for-byte identical to
 * `io/gimbal/gimbal.hpp` in sp_vision_25. That header is the single source of
 * truth; this file is a mirror of it, and the static_asserts below are the
 * contract that keeps the two sides from silently drifting apart.
 *
 * The upstream `sp_middleware/io/vision/vision.hpp` defines structures with the
 * SAME names but different layouts (it adds a `target` byte to VisionToGimbal,
 * and supercapacitor telemetry plus `game_progress` to GimbalToVision, making
 * them 30 and 57 bytes). Those layouts do NOT interoperate with the upper
 * computer: the differing CRC offset makes every frame fail its checksum. This
 * header intentionally follows sp_vision instead, which is why `io/vision` is
 * not used by this project.
 */

namespace vision
{

/// Frame header shared by both directions: ASCII "SP".
constexpr uint8_t HEAD_0 = 'S';
constexpr uint8_t HEAD_1 = 'P';

/**
 * @brief Control command, upper computer -> this board (29 bytes).
 *
 * Emitted by `io::Gimbal::send()` in sp_vision.
 */
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2];  ///< always {'S', 'P'}
  uint8_t mode;     ///< 0: no control, 1: control gimbal, 2: control + fire
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;  ///< over all preceding bytes, little-endian on the wire
};

/**
 * @brief Telemetry, this board -> upper computer (43 bytes).
 *
 * Parsed by `io::Gimbal::read_thread()` in sp_vision.
 */
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2];  ///< always {'S', 'P'}
  uint8_t mode;     ///< 0: idle, 1: auto aim, 2: small buff, 3: big buff
  float q[4];       ///< gimbal-to-world rotation, quaternion in wxyz order
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;    ///< m/s
  uint16_t bullet_count;  ///< cumulative shots fired
  uint16_t crc16;        ///< over all preceding bytes, little-endian on the wire
};

/// Command mode field values.
enum CommandMode : uint8_t
{
  MODE_NO_CONTROL = 0,
  MODE_CONTROL = 1,
  MODE_CONTROL_AND_FIRE = 2,
};

/// Telemetry mode field values, matching sp_vision's `io::GimbalMode`.
enum GimbalMode : uint8_t
{
  GIMBAL_IDLE = 0,
  GIMBAL_AUTO_AIM = 1,
  GIMBAL_SMALL_BUFF = 2,
  GIMBAL_BIG_BUFF = 3,
};

/// Wire sizes. These mirror the offsets the upper computer uses.
constexpr std::size_t VISION_TO_GIMBAL_SIZE = 29;
constexpr std::size_t GIMBAL_TO_VISION_SIZE = 43;
constexpr std::size_t VISION_TO_GIMBAL_CRC_OFFSET = 27;
constexpr std::size_t GIMBAL_TO_VISION_CRC_OFFSET = 41;

static_assert(sizeof(VisionToGimbal) == VISION_TO_GIMBAL_SIZE, "VisionToGimbal layout drift");
static_assert(sizeof(GimbalToVision) == GIMBAL_TO_VISION_SIZE, "GimbalToVision layout drift");
static_assert(
  offsetof(VisionToGimbal, crc16) == VISION_TO_GIMBAL_CRC_OFFSET,
  "VisionToGimbal crc16 offset drift");
static_assert(
  offsetof(GimbalToVision, crc16) == GIMBAL_TO_VISION_CRC_OFFSET,
  "GimbalToVision crc16 offset drift");

}  // namespace vision

#endif  // APPLICATIONS__VISION__PROTOCOL_HPP
