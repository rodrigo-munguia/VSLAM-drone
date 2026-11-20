/*---------------------------------------------------
Rodrigo Munguía 2021.

System parameters
-----------------------------------------------------
*/
#include <string>
#include <armadillo>

#ifndef PARAMETERS_GL_H
#define PARAMETERS_GL_H

using namespace std;



struct parameters
{

    // Monocular camera parameters
    int Mono_cam_img_rows;
    int Mono_cam_img_cols;
    std::vector<double> Mono_cam_distortions;
    double Mono_cam_cc_u;
    double Mono_cam_cc_v;
    double Mono_cam_fc_u;
    double Mono_cam_fc_v;
    double Mono_cam_alpha_c;

    // visual match
    int VM_max_inov_error;

    // Initialization of global map points (anchors)
    bool Init_use_anms_for_select_strong_points;
    double Init_anms_tolerance;
    int Init_number_candidate_points_per_kf;
    int Init_min_distance_to_previous_mapped_features; // pixels
    double Init_min_depth_to_consider; // meters
    // GSLAM-INIT-D4: optional birth two-view reprojection gate (default off).
    bool Init_birth_reproj_gate_enable;
    double Init_birth_reproj_max_rms_px;
    // GSLAM-BA-D6 / CONFIG-A1: relative KF pose init from last GS pose (default on).
    bool Init_relative_kf_pose_enable;
    // GSLAM-KF-Z-A5: hybrid XY-relative / Z-absolute admit (default on; requires relative).
    bool Init_hybrid_kf_z_absolute_enable;

    // Visibility Graph
    int VG_img_search_margin;

    // Delete function
    int Delete_min_kf_matched_for_keep_anchor;

    // Bundle adjusment
    int BA_max_n_previous_kf_for_search_matches;
    int BA_max_n_kf_optimized;
    int BA_min_n_latest_kf_to_optimize;
    double BA_sigma_uv;
    double BA_sigma_kf_att;
    double BA_sigma_kf_xy;
    double BA_sigma_kf_z;
    double BA_sigma_kf_pt;
    bool BA_update_kf_pos;
    bool BA_update_kf_att;
    bool BA_update_map;
    double BA_max_delta_kf_pos;


    // close loop
    int CL_min_n_not_vl_kf;
    int CL_min_n_matches;
    double CL_min_mean_reprojection_error;
    double CL_xy_update_max_delta;
    double CL_odo_sigma_kf_att;
    double CL_odo_sigma_kf_xy;
    double CL_odo_sigma_kf_z;
    double CL_clo_sigma_kf_att;
    double CL_clo_sigma_kf_xy;
    double CL_clo_sigma_kf_z;
    bool CL_update_z;
    bool CL_use_robot_uncertainty;
    bool CL_use_only_home_kf;
    /** A2d: max PnP verifies per visual Step on ranked candidates. */
    int CL_visual_max_pnp_attempts;
    /** A3g: min KF-chain traveled distance (m) for visual LC candidates. */
    double CL_min_historical_travel_m;
    double CL_tag_max_xy_dis;
    int CL_tag_max_xy_dis_px;

    // G6b.3 home-tag position capture (RecHome)
    int CL_home_tag_id;
    double CL_home_tag_size_m;
    int CL_home_capture_min_inliers;
    double CL_home_capture_min_inlier_fraction;
    double CL_home_capture_max_spread_m;
    int CL_home_capture_max_samples;

    // G6d: global processing per source (feedback flags on G2RunControl.sources)
    bool Enable_global_ba;
    bool Enable_tag_loop_closure;
    bool Enable_visual_loop_closure;

    bool Stats;
    int GM_factor_anchors_sent_to_plot;

} ;




#endif /*PARAMETERS_H*/
