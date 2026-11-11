# Latched Local SLAM notice that a correction mutation completed and a
# post-correction public odometry snapshot was published (G6a/G6b).
# Consumers must match correction_id + snapshot_seq. A newer Odometry stamp
# alone is insufficient (cross-topic ordering is not guaranteed; stamps can
# collide). Pair with ~/odometry_meta of the same snapshot_seq.

std_msgs/Header header
unique_identifier_msgs/UUID session_id
uint64 epoch_id
uint64 correction_id
uint64 snapshot_seq
string type
# G6d origin: "ba" | "tag_loop" | "visual_loop" (empty = inferred from type)
string source
bool applied
float64 applied_delta_x
float64 applied_delta_y
float64 applied_delta_z
string outcome
