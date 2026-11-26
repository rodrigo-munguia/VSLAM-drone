/**
 * Shared stable landmark identity (GlobalSLAM production; KEYFRAME observations).
 * Strongly typed — not implicitly convertible to/from storage indices.
 *
 * Convention: value 0 = invalid/unassigned; valid IDs allocate from 1.
 */
#ifndef EKF_COMMON_LANDMARK_ID_HPP
#define EKF_COMMON_LANDMARK_ID_HPP

#include <cstdint>
#include <cstddef>
#include <functional>

struct LandmarkId
{
  std::uint64_t value{0};
};

struct LandmarkIdHash
{
  std::size_t operator()(LandmarkId id) const noexcept
  {
    return static_cast<std::size_t>(id.value);
  }
};

inline bool operator==(LandmarkId a, LandmarkId b)
{
  return a.value == b.value;
}
inline bool operator!=(LandmarkId a, LandmarkId b) { return !(a == b); }

namespace std
{
template <>
struct hash<LandmarkId>
{
  std::size_t operator()(LandmarkId id) const noexcept
  {
    return static_cast<std::size_t>(id.value);
  }
};
}  // namespace std

#endif
