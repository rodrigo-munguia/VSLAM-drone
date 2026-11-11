#include "globalslam_component.hpp"

#include <cstdlib>
#include <string>

namespace globalslam
{

void Gslam::setParameters()
{
  this->declare_parameter<int>("Mono_cam_img_rows", 240);
  this->declare_parameter<int>("Mono_cam_img_cols", 320);
  this->declare_parameter(
    "Mono_cam_distortions",
    std::vector<double>{0.02921, -0.00504, 0.00297, -0.00843, 0.00000});
  this->declare_parameter<double>("Mono_cam_cc_u", 156.24435);
  this->declare_parameter<double>("Mono_cam_cc_v", 117.04562);
  this->declare_parameter<double>("Mono_cam_fc_u", 206.34225);
  this->declare_parameter<double>("Mono_cam_fc_v", 268.65192);
  this->declare_parameter<double>("Mono_cam_alpha_c", 0.0);
  this->declare_parameter<int>("Init_number_candidate_points_per_kf", 500);
  this->declare_parameter<bool>("Init_use_anms_for_select_strong_points", true);
  this->declare_parameter<double>("Init_anms_tolerance", 0.3);
  this->declare_parameter<int>("Init_min_distance_to_previous_mapped_features", 5);
  this->declare_parameter<double>("Init_min_depth_to_consider", 0.2);
  // Birth reprojection gate (INIT-D4). Enabled by default after D2–D6 validation; 20 px RMS.
  // Disable via ROS param or GSLAM_INIT_BIRTH_REPROJ_GATE_ENABLE=0.
  this->declare_parameter<bool>("Init_birth_reproj_gate_enable", true);
  this->declare_parameter<double>("Init_birth_reproj_max_rms_px", 20.0);
  // Relative KF pose init (BA-D6). Disabled by default; env override for A/B.
  this->declare_parameter<bool>("Init_relative_kf_pose_enable", false);
  this->declare_parameter<int>("VM_max_inov_error", 20);
  this->declare_parameter<int>("VG_img_search_margin", 15);
  this->declare_parameter<int>("Delete_min_kf_matched_for_keep_anchor", 3);
  this->declare_parameter<int>("BA_max_n_previous_kf_for_search_matches", 20);
  this->declare_parameter<int>("BA_max_n_kf_optimized", 7);
  this->declare_parameter<int>("BA_min_n_latest_kf_to_optimize", 4);
  this->declare_parameter<double>("BA_sigma_uv", 1.0);
  this->declare_parameter<double>("BA_sigma_kf_att", 0.01);
  this->declare_parameter<double>("BA_sigma_kf_xy", 0.1);
  this->declare_parameter<double>("BA_sigma_kf_z", 0.1);
  this->declare_parameter<double>("BA_sigma_kf_pt", 0.2);
  this->declare_parameter<bool>("BA_update_kf_pos", true);
  this->declare_parameter<bool>("BA_update_kf_att", true);
  this->declare_parameter<bool>("BA_update_map", true);
  this->declare_parameter<double>("BA_max_delta_kf_pos", 0.2);
  this->declare_parameter<int>("CL_min_n_not_vl_kf", 2);
  this->declare_parameter<int>("CL_min_n_matches", 10);
  this->declare_parameter<double>("CL_min_mean_reprojection_error", 25.0);
  this->declare_parameter<double>("CL_xy_update_max_delta", 6.0);
  this->declare_parameter<double>("CL_odo_sigma_kf_att", 0.0001);
  this->declare_parameter<double>("CL_odo_sigma_kf_xy", 0.05);
  this->declare_parameter<double>("CL_odo_sigma_kf_z", 0.05);
  this->declare_parameter<double>("CL_clo_sigma_kf_att", 0.00001);
  this->declare_parameter<double>("CL_clo_sigma_kf_xy", 0.005);
  this->declare_parameter<double>("CL_clo_sigma_kf_z", 0.005);
  this->declare_parameter<bool>("CL_update_z", true);
  this->declare_parameter<bool>("CL_use_robot_uncertainty", true);
  this->declare_parameter<bool>("CL_use_only_home_kf", false);
  this->declare_parameter<bool>("Stats", false);
  this->declare_parameter<double>("CL_tag_max_xy_dis", 4.0);
  this->declare_parameter<int>("CL_tag_max_xy_dis_px", 100);
  this->declare_parameter<int>("CL_home_tag_id", 23);
  this->declare_parameter<double>("CL_home_tag_size_m", 1.0);
  this->declare_parameter<int>("CL_home_capture_min_inliers", 3);
  this->declare_parameter<double>("CL_home_capture_min_inlier_fraction", 0.5);
  this->declare_parameter<double>("CL_home_capture_max_spread_m", 1.5);
  this->declare_parameter<int>("CL_home_capture_max_samples", 40);
  this->declare_parameter<int>("GM_factor_anchors_sent_to_plot", 1);
  // G6d: processing defaults preserve prior BA+tag behavior; visual off until validated.
  // All Local SLAM feedback defaults off. Master Apply_global_corrections forces
  // apply_* only for enabled sources (never enables visual processing alone).
  this->declare_parameter<bool>("Enable_global_ba", true);
  this->declare_parameter<bool>("Enable_tag_loop_closure", true);
  this->declare_parameter<bool>("Enable_visual_loop_closure", false);
  this->declare_parameter<bool>("Apply_ba_corrections", false);
  this->declare_parameter<bool>("Apply_tag_loop_corrections", false);
  this->declare_parameter<bool>("Apply_visual_loop_corrections", false);
  this->declare_parameter<bool>("Apply_global_corrections", false);
  this->declare_parameter<bool>("Processing_start_enabled", false);
  this->declare_parameter<bool>("Accept_missing_source_stamp", false);
  this->declare_parameter<std::string>(
    "Estimator_reference_topic", "/EKFslam/estimator_reference");
  // G4 public visualization (same parent frame as Local SLAM odometry).
  this->declare_parameter<std::string>("Public_frame_id", "odom");
  // 0 = publish full committed snapshot (no viz truncation).
  this->declare_parameter<int>("Path_max_poses", 0);
  this->declare_parameter<int>("Landmark_cloud_max_points", 0);
  // G6a coordination (dormant while no apply_* feedback is enabled).
  this->declare_parameter<std::string>("Coordinate_mode", "required");
  this->declare_parameter<int>("Coordinate_pause_timeout_ms", 2000);
  this->declare_parameter<int>("Coordinate_feedback_barrier_timeout_ms", 2000);
  this->declare_parameter<std::string>(
    "Correction_applied_topic", "/EKFslam/correction_applied");

  this->get_parameter("Mono_cam_img_rows", PAR.Mono_cam_img_rows);
  this->get_parameter("Mono_cam_img_cols", PAR.Mono_cam_img_cols);
  rclcpp::Parameter Mono_cam_distortions_par = this->get_parameter("Mono_cam_distortions");
  PAR.Mono_cam_distortions = Mono_cam_distortions_par.as_double_array();
  this->get_parameter("Mono_cam_cc_u", PAR.Mono_cam_cc_u);
  this->get_parameter("Mono_cam_cc_v", PAR.Mono_cam_cc_v);
  this->get_parameter("Mono_cam_fc_u", PAR.Mono_cam_fc_u);
  this->get_parameter("Mono_cam_fc_v", PAR.Mono_cam_fc_v);
  this->get_parameter("Mono_cam_alpha_c", PAR.Mono_cam_alpha_c);
  this->get_parameter("Init_number_candidate_points_per_kf", PAR.Init_number_candidate_points_per_kf);
  this->get_parameter(
    "Init_use_anms_for_select_strong_points", PAR.Init_use_anms_for_select_strong_points);
  this->get_parameter("Init_anms_tolerance", PAR.Init_anms_tolerance);
  this->get_parameter(
    "Init_min_distance_to_previous_mapped_features",
    PAR.Init_min_distance_to_previous_mapped_features);
  this->get_parameter("Init_min_depth_to_consider", PAR.Init_min_depth_to_consider);
  this->get_parameter("Init_birth_reproj_gate_enable", PAR.Init_birth_reproj_gate_enable);
  this->get_parameter("Init_birth_reproj_max_rms_px", PAR.Init_birth_reproj_max_rms_px);
  this->get_parameter("Init_relative_kf_pose_enable", PAR.Init_relative_kf_pose_enable);
  // Optional env override for controlled A/B without YAML/default changes:
  //   GSLAM_INIT_BIRTH_REPROJ_GATE_ENABLE=1|0
  //   GSLAM_INIT_BIRTH_REPROJ_MAX_RMS_PX=<float>
  //   GSLAM_INIT_RELATIVE_KF_POSE=1|0
  if (const char * e = std::getenv("GSLAM_INIT_BIRTH_REPROJ_GATE_ENABLE")) {
    if (e[0] != '\0') {
      PAR.Init_birth_reproj_gate_enable =
        (e[0] == '1' || e[0] == 't' || e[0] == 'T' || e[0] == 'y' || e[0] == 'Y');
    }
  }
  if (const char * e = std::getenv("GSLAM_INIT_BIRTH_REPROJ_MAX_RMS_PX")) {
    if (e[0] != '\0') {
      try {
        PAR.Init_birth_reproj_max_rms_px = std::stod(e);
      } catch (...) {
      }
    }
  }
  if (const char * e = std::getenv("GSLAM_INIT_RELATIVE_KF_POSE")) {
    if (e[0] != '\0') {
      PAR.Init_relative_kf_pose_enable =
        (e[0] == '1' || e[0] == 't' || e[0] == 'T' || e[0] == 'y' || e[0] == 'Y');
    }
  }
  this->get_parameter("VM_max_inov_error", PAR.VM_max_inov_error);
  this->get_parameter("VG_img_search_margin", PAR.VG_img_search_margin);
  this->get_parameter("Delete_min_kf_matched_for_keep_anchor", PAR.Delete_min_kf_matched_for_keep_anchor);
  this->get_parameter(
    "BA_max_n_previous_kf_for_search_matches", PAR.BA_max_n_previous_kf_for_search_matches);
  this->get_parameter("BA_max_n_kf_optimized", PAR.BA_max_n_kf_optimized);
  this->get_parameter("BA_min_n_latest_kf_to_optimize", PAR.BA_min_n_latest_kf_to_optimize);
  this->get_parameter("BA_sigma_uv", PAR.BA_sigma_uv);
  this->get_parameter("BA_sigma_kf_att", PAR.BA_sigma_kf_att);
  this->get_parameter("BA_sigma_kf_xy", PAR.BA_sigma_kf_xy);
  this->get_parameter("BA_sigma_kf_z", PAR.BA_sigma_kf_z);
  this->get_parameter("BA_sigma_kf_pt", PAR.BA_sigma_kf_pt);
  this->get_parameter("BA_update_kf_pos", PAR.BA_update_kf_pos);
  this->get_parameter("BA_update_kf_att", PAR.BA_update_kf_att);
  this->get_parameter("BA_update_map", PAR.BA_update_map);
  this->get_parameter("BA_max_delta_kf_pos", PAR.BA_max_delta_kf_pos);
  this->get_parameter("CL_min_n_not_vl_kf", PAR.CL_min_n_not_vl_kf);
  this->get_parameter("CL_min_n_matches", PAR.CL_min_n_matches);
  this->get_parameter("CL_min_mean_reprojection_error", PAR.CL_min_mean_reprojection_error);
  this->get_parameter("CL_xy_update_max_delta", PAR.CL_xy_update_max_delta);
  this->get_parameter("CL_odo_sigma_kf_att", PAR.CL_odo_sigma_kf_att);
  this->get_parameter("CL_odo_sigma_kf_xy", PAR.CL_odo_sigma_kf_xy);
  this->get_parameter("CL_odo_sigma_kf_z", PAR.CL_odo_sigma_kf_z);
  this->get_parameter("CL_clo_sigma_kf_att", PAR.CL_clo_sigma_kf_att);
  this->get_parameter("CL_clo_sigma_kf_xy", PAR.CL_clo_sigma_kf_xy);
  this->get_parameter("CL_clo_sigma_kf_z", PAR.CL_clo_sigma_kf_z);
  this->get_parameter("CL_update_z", PAR.CL_update_z);
  this->get_parameter("CL_use_robot_uncertainty", PAR.CL_use_robot_uncertainty);
  this->get_parameter("CL_use_only_home_kf", PAR.CL_use_only_home_kf);
  this->get_parameter("Stats", PAR.Stats);
  this->get_parameter("CL_tag_max_xy_dis", PAR.CL_tag_max_xy_dis);
  this->get_parameter("CL_tag_max_xy_dis_px", PAR.CL_tag_max_xy_dis_px);
  this->get_parameter("CL_home_tag_id", PAR.CL_home_tag_id);
  this->get_parameter("CL_home_tag_size_m", PAR.CL_home_tag_size_m);
  this->get_parameter("CL_home_capture_min_inliers", PAR.CL_home_capture_min_inliers);
  this->get_parameter(
    "CL_home_capture_min_inlier_fraction", PAR.CL_home_capture_min_inlier_fraction);
  this->get_parameter("CL_home_capture_max_spread_m", PAR.CL_home_capture_max_spread_m);
  this->get_parameter("CL_home_capture_max_samples", PAR.CL_home_capture_max_samples);
  this->get_parameter("GM_factor_anchors_sent_to_plot", PAR.GM_factor_anchors_sent_to_plot);

  this->get_parameter("Enable_global_ba", PAR.Enable_global_ba);
  this->get_parameter("Enable_tag_loop_closure", PAR.Enable_tag_loop_closure);
  this->get_parameter("Enable_visual_loop_closure", PAR.Enable_visual_loop_closure);
  bool apply_ba = false;
  bool apply_tag = false;
  bool apply_vis = false;
  bool apply_corrections = false;
  bool start_enabled = false;
  this->get_parameter("Apply_ba_corrections", apply_ba);
  this->get_parameter("Apply_tag_loop_corrections", apply_tag);
  this->get_parameter("Apply_visual_loop_corrections", apply_vis);
  this->get_parameter("Apply_global_corrections", apply_corrections);
  this->get_parameter("Processing_start_enabled", start_enabled);
  this->get_parameter("Accept_missing_source_stamp", accept_missing_source_stamp_);
  this->get_parameter("Estimator_reference_topic", estimator_reference_topic_);
  this->get_parameter("Public_frame_id", public_frame_id_);
  this->get_parameter("Path_max_poses", path_max_poses_);
  this->get_parameter("Landmark_cloud_max_points", landmark_cloud_max_points_);
  this->get_parameter("Coordinate_mode", coordinate_mode_str_);
  this->get_parameter("Coordinate_pause_timeout_ms", coordinate_pause_timeout_ms_);
  this->get_parameter(
    "Coordinate_feedback_barrier_timeout_ms", coordinate_feedback_barrier_timeout_ms_);
  this->get_parameter("Correction_applied_topic", correction_applied_topic_);

  run_ctrl_.sources.processing_enabled = start_enabled;
  run_ctrl_.sources.enable_global_ba = PAR.Enable_global_ba;
  run_ctrl_.sources.enable_tag_loop_closure = PAR.Enable_tag_loop_closure;
  run_ctrl_.sources.enable_visual_loop_closure = PAR.Enable_visual_loop_closure;
  run_ctrl_.sources.apply_ba_corrections = apply_ba;
  run_ctrl_.sources.apply_tag_loop_corrections = apply_tag;
  run_ctrl_.sources.apply_visual_loop_corrections = apply_vis;
  run_ctrl_.sources.apply_global_corrections_master = apply_corrections;
  run_ctrl_.sources.apply_master_compatibility();
  std::string pol_reason;
  if (!run_ctrl_.sources.validate(pol_reason)) {
    RCLCPP_FATAL(this->get_logger(), "G6d source policy invalid: %s", pol_reason.c_str());
    throw std::runtime_error("G6d source policy invalid: " + pol_reason);
  }
  RCLCPP_INFO(
    this->get_logger(), "G6d source matrix: %s",
    run_ctrl_.sources.matrix_summary().c_str());
}

}  // namespace globalslam
