#include "cloop.hpp"
#include "cloop_visual_pnp.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"

#include <cstring>

// Optional sink for last accepted visual PnP inlier correspondence indices
// (into idx_pt_matches / image_points). Cleared on entry; filled on accept.
static thread_local std::vector<int> * g_last_visual_inlier_indices = nullptr;

void CLOOP_set_visual_inlier_sink(std::vector<int> * sink)
{
  g_last_visual_inlier_indices = sink;
}

bool CLOOP::Get_pos_of_current_kf(
  arma::vec::fixed<3> & pos,
  KEYFRAME & kf_cl,
  std::vector<int> & idx_pt_matches,
  std::vector<cv::Point2d> & image_points,
  int step)
{
  if (g_last_visual_inlier_indices) {
    g_last_visual_inlier_indices->clear();
  }
  std::vector<cv::Point3f> Points3D;
  std::vector<cv::Point2f> Points2D;
  Points3D.reserve(idx_pt_matches.size());
  Points2D.reserve(idx_pt_matches.size());

  for (size_t i = 0; i < idx_pt_matches.size(); ++i) {
    const int64 idx_pt = idx_pt_matches[i];
    const double x = Gmap.AnchorsDATA[idx_pt].AnchorState[0];
    const double y = Gmap.AnchorsDATA[idx_pt].AnchorState[1];
    const double z = Gmap.AnchorsDATA[idx_pt].AnchorState[2];
    Points3D.emplace_back(
      static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));

    // Undistort into the domain expected by solvePnP (distCoeffs = 0 there).
    const cv::Point2d uv = Undistort_a_point(image_points[i], cam_parameters, 1);
    Points2D.emplace_back(static_cast<float>(uv.x), static_cast<float>(uv.y));
  }

  // A2b: seed R_cv=Rn2c, t_cv=-Rn2c*C; gate on PnP extrinsics; C=-R^T t;
  // inliers as OpenCV4 index list. See cloop_visual_pnp.hpp.
  const auto solved = globalslam::cloop_visual_pnp::solve_and_accept(
    Points3D,
    Points2D,
    kf_cl.Rn2c,
    kf_cl.t_c2n,
    cam_parameters,
    kf_cl.radius_uncertainty,
    PAR.CL_xy_update_max_delta,
    PAR.CL_min_mean_reprojection_error);

  // Count an evaluation attempt whenever RANSAC returned a pose to score
  // (including gate failures); matches prior n_cloop_intents accounting.
  if (std::strcmp(solved.reject_reason, "insufficient_correspondences") != 0 &&
    std::strcmp(solved.reject_reason, "empty_inliers") != 0 &&
    std::strcmp(solved.reject_reason, "unexpected_inliers_type") != 0)
  {
    n_cloop_intents++;
    cout << "Potential Loop: Mean reprojection error: "
         << solved.mean_reprojection_error
         << " (inliers=" << solved.n_inliers
         << " d_xy=" << solved.d_xy << ")" << endl;
  }

  if (!solved.accepted) {
    return false;
  }

  // Downstream Pose_SLAM consumes camera centre in nav only (not PnP R).
  pos = solved.C_nav;
  if (g_last_visual_inlier_indices) {
    *g_last_visual_inlier_indices = solved.inlier_indices;
  }

  if (PAR.Stats) {
    CLOOP_Stat cloop_stat;
    cloop_stat.n_loop_closing_intents = n_cloop_intents;
    cloop_stat.reprojection_error = solved.mean_reprojection_error;
    cloop_stat.error_correction = solved.d_xy;
    cloop_stat.step = step;
    store.cloop_stats.push_back(cloop_stat);
  }
  return true;
}
//-----------------------------------------------------------
bool CLOOP::Detect_home_tag_on_kf(
  const KEYFRAME & kf,
  int & tag_id,
  double tvec_out[3],
  double & d_px) const
{
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters =
    cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  const double fx = cam_parameters.fc[0];
  const double fy = cam_parameters.fc[1];
  const double cx = cam_parameters.cc[0];
  const double cy = cam_parameters.cc[1];
  const double k1 = cam_parameters.distortions[0];
  const double k2 = cam_parameters.distortions[1];
  const double p1 = cam_parameters.distortions[2];
  const double p2 = cam_parameters.distortions[3];
  const double k3 = cam_parameters.distortions[4];

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);

  cv::aruco::detectMarkers(
    kf.frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
  if (markerIds.empty()) {
    return false;
  }

  int best = -1;
  for (size_t i = 0; i < markerIds.size(); ++i) {
    if (markerIds[i] == PAR.CL_home_tag_id) {
      best = static_cast<int>(i);
      break;
    }
  }
  if (best < 0) {
    return false;
  }

  // OpenCV: tvecs[i] = marker origin in camera optical frame (T_C←H).
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(
    markerCorners, PAR.CL_home_tag_size_m, cameraMatrix, distCoeffs, rvecs, tvecs);

  const std::vector<cv::Point2f> & markerCorner = markerCorners[best];
  const cv::Point2f center = (markerCorner[0] + markerCorner[2]) * 0.5f;
  d_px = std::sqrt(std::pow(cx - center.x, 2) + std::pow(cy - center.y, 2));
  tag_id = markerIds[best];
  tvec_out[0] = tvecs[best][0];
  tvec_out[1] = tvecs[best][1];
  tvec_out[2] = tvecs[best][2];
  (void)rvecs;  // orientation unused (position reference only)
  return true;
}

//-----------------------------------------------------------
bool CLOOP::Get_pos_of_current_kf_from_Home_tag(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl)
{
        using globalslam::g6b_home_tag::convert_home_tag_tvec_to_nav_camera_pos;
        using globalslam::g6b_home_tag::flatten_Rn2c_row_major;
        using globalslam::g6b_home_tag::pose_reject_name;

        const auto & home_ref = home_tag_capture_.reference();

        int tag_id = -1;
        double tvec[3]{0, 0, 0};
        double d_px = 0;
        if (!Detect_home_tag_on_kf(kf_cl, tag_id, tvec, d_px)) {
          return false;
        }
        if (tag_id != PAR.CL_home_tag_id) {
          return false;
        }

        double Rn2c_rm[9];
        flatten_Rn2c_row_major(
          kf_cl.Rn2c(0, 0), kf_cl.Rn2c(0, 1), kf_cl.Rn2c(0, 2),
          kf_cl.Rn2c(1, 0), kf_cl.Rn2c(1, 1), kf_cl.Rn2c(1, 2),
          kf_cl.Rn2c(2, 0), kf_cl.Rn2c(2, 1), kf_cl.Rn2c(2, 2),
          Rn2c_rm);

        const bool session_ok =
          home_ref.valid &&
          slam_identity::equal(home_ref.session_id, kf_cl.session_id) &&
          home_ref.epoch_id == kf_cl.epoch_id;

        double tx = 0, ty = 0, tz = 0;
        const auto reject = convert_home_tag_tvec_to_nav_camera_pos(
          tvec[0], tvec[1], tvec[2],
          Rn2c_rm,
          PAR.CL_tag_max_xy_dis,
          static_cast<double>(PAR.CL_tag_max_xy_dis_px), d_px,
          home_ref.valid, home_ref.spatial_ready, session_ok,
          home_ref.p_H_N[0], home_ref.p_H_N[1], home_ref.p_H_N[2],
          tx, ty, tz);

        if (reject != globalslam::g6b_home_tag::PoseReject::Ok) {
          cout << "cloop-> home-tag pose rejected: " << pose_reject_name(reject)
               << " (tvec=[" << tvec[0] << " " << tvec[1] << " "
               << tvec[2] << "] d_px=" << d_px << ")" << endl;
          return false;
        }

        // Absolute camera position in N for Pose_SLAM.
        // Uses frozen p_H_N; detecting-KF Rn2c (attitude-error coupling retained).
        // rvec unused. Pose_SLAM BetweenFactor R≡I remains a full 6-D factor.
        pos[0] = tx;
        pos[1] = ty;
        pos[2] = tz;
        cout << "cloop-> home-tag t_c2n (nav camera)= [" << tx << " " << ty
             << " " << tz << "] (p_H_N=[" << home_ref.p_H_N[0] << " "
             << home_ref.p_H_N[1] << " " << home_ref.p_H_N[2]
             << "] via p_H - Rn2c^T tvec)" << endl;
        return true;
}