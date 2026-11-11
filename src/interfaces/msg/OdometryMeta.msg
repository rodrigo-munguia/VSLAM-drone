# Companion metadata for a single nav_msgs/Odometry publication (G6b).
# snapshot_seq is unique per Local SLAM instance and is the association key —
# header.stamp alone is insufficient when multiple messages can share a stamp.
# Publish immediately after the paired Odometry with the same header.stamp.
# Pose remains on standard nav_msgs/Odometry only (do not overload frame_id /
# covariance / timestamps for correction identity).

std_msgs/Header header
uint64 snapshot_seq
unique_identifier_msgs/UUID session_id
uint64 epoch_id
# 0 = this snapshot is not the first after a correction; otherwise the
# correction_id that produced this (and subsequent) snapshots until the next
# applied correction or estimator reset.
uint64 last_correction_id
