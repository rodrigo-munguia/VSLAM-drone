#ifndef EKF_COMMON_SLAM_IDENTITY_HPP_
#define EKF_COMMON_SLAM_IDENTITY_HPP_

/**
 * Shared Local↔Global SLAM identity helpers (session UUID, comparisons).
 * Header-only; no ROS message dependency so unit tests stay lightweight.
 */

#include <array>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>

namespace slam_identity
{

using UuidBytes = std::array<uint8_t, 16>;

inline UuidBytes nil_uuid()
{
  UuidBytes u{};
  u.fill(0);
  return u;
}

inline bool is_nil(const UuidBytes & u)
{
  for (uint8_t b : u) {
    if (b != 0) {
      return false;
    }
  }
  return true;
}

inline bool equal(const UuidBytes & a, const UuidBytes & b)
{
  return std::memcmp(a.data(), b.data(), 16) == 0;
}

/** RFC-4122 variant-1 random UUID (version 4). */
inline UuidBytes random_uuid_v4()
{
  UuidBytes u{};
  std::random_device rd;
  std::uniform_int_distribution<int> dist(0, 255);
  for (size_t i = 0; i < 16; ++i) {
    u[i] = static_cast<uint8_t>(dist(rd));
  }
  // version 4
  u[6] = static_cast<uint8_t>((u[6] & 0x0f) | 0x40);
  // variant 1
  u[8] = static_cast<uint8_t>((u[8] & 0x3f) | 0x80);
  return u;
}

inline std::string to_string(const UuidBytes & u)
{
  static const char * hex = "0123456789abcdef";
  std::string s;
  s.resize(36);
  size_t j = 0;
  for (size_t i = 0; i < 16; ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) {
      s[j++] = '-';
    }
    s[j++] = hex[(u[i] >> 4) & 0xf];
    s[j++] = hex[u[i] & 0xf];
  }
  return s;
}

struct ReferenceId
{
  UuidBytes session_id{nil_uuid()};
  uint64_t epoch_id{0};

  bool operator==(const ReferenceId & o) const
  {
    return epoch_id == o.epoch_id && equal(session_id, o.session_id);
  }

  bool operator!=(const ReferenceId & o) const { return !(*this == o); }
};

}  // namespace slam_identity

#endif  // EKF_COMMON_SLAM_IDENTITY_HPP_
