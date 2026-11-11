# Informational: a global loop result was committed to the shared Global map.
# Independent of Local SLAM feedback permission / application.
#
# Does NOT imply:
#   - Local EKF pose was corrected
#   - Pc/radius was reset
#   - Control should re-navigate
#
# Correlated application/completion use existing CorrectionApplied /
# CorrectionFeedbackAccepted / CorrectionCoordinate paths when feedback is on.

unique_identifier_msgs/UUID session_id
uint64 epoch_id
uint64 keyframe_id
uint64 map_revision
# "tag_loop" | "visual_loop"
string source
# Closing camera position in internal nav after Pose_SLAM commit (metres).
float64 committed_camera_x
float64 committed_camera_y
float64 committed_camera_z
# True if optional visual match/BA refine ran after Pose_SLAM.
bool match_ba_refine_ran
