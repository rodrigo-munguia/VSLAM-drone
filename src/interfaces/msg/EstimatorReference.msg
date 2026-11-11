# Explicit Local SLAM reference identity for adapters and Global SLAM.
# Published latched (TRANSIENT_LOCAL) so late subscribers receive the current state.
#
# Identity is (session_id, epoch_id):
#   session_id — UUID generated once per Local SLAM component instance.
#   epoch_id   — increments on each successful system_init after reinitialization.
#
# valid=false  → current epoch invalidated (reset); withhold epoch-dependent GT /
#                Global SLAM processing. session_id remains that of this instance.
# valid=true   → (session_id, epoch_id) identifies the current initialization.
#
# Disable/re-enable does not change session_id or epoch_id.
# Same-epoch altitude/heading metadata updates republish without a new epoch.
#
# Assumption: one authoritative Local SLAM EstimatorReference publisher per
# configured input. Consumers should reject delayed old-session data after a
# newer session has been adopted.
#
# yaw_at_home_internal is exactly the Att.yaw value stored by EKF (pitch/yaw
# already in the internal convention). yaw_enu_equivalent = -yaw_at_home_internal
# is the public/Webots ENU yaw corresponding to that noisy measurement.

std_msgs/Header header

unique_identifier_msgs/UUID session_id
uint64 epoch_id
bool valid

# Configured initial position in the *internal* nav frame (metres).
float64 robot_init_x
float64 robot_init_y
float64 robot_init_z

# Heading reference from the attitude sample accepted at system_init.
bool heading_valid
builtin_interfaces/Time heading_stamp
float64 yaw_at_home_internal
float64 yaw_enu_equivalent

# Altitude home (first altitude update of this epoch), or immediately ready
# when Altitude_update is disabled (then altitude_stamp mirrors heading_stamp
# and altitude_home_up is unused / NaN).
bool altitude_update_enabled
bool altitude_valid
builtin_interfaces/Time altitude_stamp
float64 altitude_home_up
