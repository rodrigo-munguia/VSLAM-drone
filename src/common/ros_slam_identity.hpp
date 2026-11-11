#ifndef EKF_COMMON_ROS_SLAM_IDENTITY_HPP_
#define EKF_COMMON_ROS_SLAM_IDENTITY_HPP_

/**
 * Convert between slam_identity::UuidBytes and unique_identifier_msgs/UUID.
 */

#include "slam_identity.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace slam_identity
{

inline UuidBytes from_msg(const unique_identifier_msgs::msg::UUID & msg)
{
  UuidBytes u{};
  for (size_t i = 0; i < 16; ++i) {
    u[i] = msg.uuid[i];
  }
  return u;
}

inline unique_identifier_msgs::msg::UUID to_msg(const UuidBytes & u)
{
  unique_identifier_msgs::msg::UUID msg;
  for (size_t i = 0; i < 16; ++i) {
    msg.uuid[i] = u[i];
  }
  return msg;
}

}  // namespace slam_identity

#endif  // EKF_COMMON_ROS_SLAM_IDENTITY_HPP_
