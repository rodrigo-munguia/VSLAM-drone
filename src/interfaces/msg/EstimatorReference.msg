# Explicit Local SLAM reference epoch for ground-truth adapters.
# Published latched (TRANSIENT_LOCAL) so late subscribers receive the current epoch.
#
# valid=false  → previous epoch invalidated (reset); withhold epoch-dependent GT.
# valid=true   → epoch_id identifies the current initialization.
#
# Heading and altitude references may become valid at different times; check
# heading_valid / altitude_valid separately. Do not assume simultaneity.
#
# yaw_at_home_internal is exactly the Att.yaw value stored by EKF (pitch/yaw
# already in the internal convention). yaw_enu_equivalent = -yaw_at_home_internal
# is the public/Webots ENU yaw corresponding to that noisy measurement — the
# value GT adapters must use for R_z(-yaw_enu) so heading matches the estimator.

std_msgs/Header header

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
