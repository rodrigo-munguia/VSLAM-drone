#ifndef EKF_COMMON_KEYFRAME_TYPES_H
#define EKF_COMMON_KEYFRAME_TYPES_H

/**
 * Shared KEYFRAME type used by Local SLAM (producer) and Global SLAM (consumer).
 * Kept outside globalslam so localslam does not include the Global SLAM package.
 */

#include <armadillo>
#include <opencv2/opencv.hpp>
#include <vector>

struct KEYFRAME
{
  cv::Mat frame;
  arma::vec::fixed<3> t_c2r;
  arma::mat::fixed<3, 3> Rr2c;
  arma::vec::fixed<3> r_N;
  arma::mat::fixed<3, 3> Rn2r;
  arma::mat::fixed<3, 3> Rn2c;
  arma::vec::fixed<3> t_c2n;
  std::vector<cv::KeyPoint> keyPoints_all;
  cv::Mat Descriptors_all;
  // OpenCV typedef int64 (same as historical globalslam_types.hpp field).
  std::vector<int64> Idx_Matched_points;
  std::vector<cv::Point2d> UV_Matched_points;
  double radius_uncertainty;
};

#endif
