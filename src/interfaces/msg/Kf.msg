# Atomic Local SLAM keyframe packet for Global SLAM (map + close-loop streams).
#
# Coordinate semantics (NOT public odom / base_link):
#   r2c — camera pose relative to the robot in the estimator's internal frames
#         (translation = t_c2r, rotation = Rr2c).
#   n2r — robot pose in the internal navigation frame (translation = r_N,
#         rotation = Rn2r). Navigation z is down-positive as used by the EKF.
#
# Identity / timing:
#   session_id, epoch_id — captured at keyframe snapshot creation (not at publish).
#   keyframe_id — monotonic per (session_id, epoch_id), shared across STREAM_MAP
#                 and STREAM_CLOSE_LOOP. Map and close-loop selection are mutually
#                 exclusive for a given image frame; IDs never collide across streams.
#   header.stamp / img.header.stamp — source image timestamp when
#                 source_stamp_valid=true. Never fabricated from publish time.
#   source_stamp_valid=false — legacy/headerless/zero-stamp inputs; consumers
#                 must apply an explicit missing-stamp policy (default: reject).

std_msgs/Header header

unique_identifier_msgs/UUID session_id
uint64 epoch_id
uint64 keyframe_id
bool source_stamp_valid

uint8 STREAM_MAP=0
uint8 STREAM_CLOSE_LOOP=1
uint8 stream

sensor_msgs/Image img
geometry_msgs/Transform r2c
geometry_msgs/Transform n2r
float64 radius_uncertainty
