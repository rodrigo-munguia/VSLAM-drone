# Control notice that the post-correction feedback barrier is satisfied (G6b).
# Global SLAM must wait for this (when coordinating) before release/resume under
# odometry feedback. GT feedback may accept on CorrectionApplied without
# consuming corrected odometry (pose source is GPS/Webots state).

std_msgs/Header header
unique_identifier_msgs/UUID session_id
uint64 epoch_id
uint64 correction_id
uint64 snapshot_seq
# "odometry" | "gt" | "none"
string feedback_mode
