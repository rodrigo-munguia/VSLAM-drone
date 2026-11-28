#ifndef EKF_COMMON_KEYFRAME_TYPES_H
#define EKF_COMMON_KEYFRAME_TYPES_H

/**
 * Shared KEYFRAME type used by Local SLAM (producer) and Global SLAM (consumer).
 * Kept outside globalslam so localslam does not include the Global SLAM package.
 *
 * Identity/timing fields are captured when the snapshot is created (not at ROS
 * publish). Coordinates remain the estimator's internal conventions.
 */

#include <armadillo>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <vector>

#include "landmark_id.hpp"
#include "slam_identity.hpp"

struct KEYFRAME
{
  cv::Mat frame;
  arma::vec::fixed<3> t_c2r;
  arma::mat::fixed<3, 3> Rr2c;
  arma::vec::fixed<3> r_N;
  arma::mat::fixed<3, 3> Rn2r;
  arma::mat::fixed<3, 3> Rn2c;
  arma::vec::fixed<3> t_c2n;
  /**
   * M7.7.2: immutable LocalSLAM camera centre at GMAP admission, captured
   * before relative/hybrid Global pose rewrite. Never updated by BA / Pose_SLAM
   * / post-loop BA. Used for δ_L = p_G_final − p_L_original and travel gates.
   */
  arma::vec::fixed<3> t_c2n_local_original{};
  bool local_original_valid{false};
  std::vector<cv::KeyPoint> keyPoints_all;
  cv::Mat Descriptors_all;
  // Persistent GlobalSLAM landmark associations (stable IDs; not storage indices).
  std::vector<LandmarkId> Matched_landmark_ids;
  std::vector<cv::Point2d> UV_Matched_points;
  double radius_uncertainty{0.0};

  // G3 identity / timing (captured at selection).
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  uint64_t keyframe_id{0};
  long int source_time_ms{0};
  bool source_stamp_valid{false};
  uint8_t stream{0};  // 0=map, 1=close_loop
};

#endif
