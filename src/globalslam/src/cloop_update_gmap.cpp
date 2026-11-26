#include "cloop.hpp"
#include "g6b_cl_match_index.hpp"
#include "gmap_graph_correction.hpp"
#include "gmap_visual_sync.hpp"


void CLOOP::Update_gmap(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl,int &idx_kf_matched)
{
     last_pose_slam_committed_ = false;
     last_match_ba_refine_ran_ = false;

     // LC-CONSISTENCY-A3: closing KF arrives from Local with absolute EKF
     // camera pose. CLOOP does not run relative/hybrid rewrite on kf_cl, so
     // t_c2n here is p_L_original for alignment feedback (not a GS-refined pose).
     const arma::vec::fixed<3> p_l_original = kf_cl.t_c2n;
     const uint64_t sync_kf_id = kf_cl.keyframe_id;

     // Add the close loop frame as a new keyFrame in the global map
     Gmap.KeyFDATA.push_back(kf_cl);
     //------------- Try to match the global map points in the new kf
     int idx_KF = Gmap.KeyFDATA.size()-1;
     Match_Gmap_points_into_KF(idx_KF);
     //------------- Update the visibility graph for taking into account the new keyframe
     Update_Visibility_Graph_with_new_KF(idx_KF);


     std::vector<arma::vec::fixed<3>> delta_kf_pos;
     std::vector<int> idx_kf_opt;

     Pose_SLAM(pos,delta_kf_pos,idx_kf_opt);

     if (delta_kf_pos.empty() || idx_kf_opt.empty()) {
       cout << "cloop-> Pose_SLAM produced no deltas; aborting map update "
            << "(ref=" << Gmap.idx_ref_pose_slam
            << " n_kf=" << Gmap.KeyFDATA.size() << ")" << endl;
       // Keep the appended KF/visibility updates; do not touch Delta_kf_n.
       return;
     }

    // M7.3: explicit-map landmark/KF geometry apply (same equations as before).
    gmap_graph_correction::apply_pose_slam_geometry(
      Gmap, delta_kf_pos, idx_kf_opt);

    // Pose-graph reference used as home-tag matched-KF fallback (before bump).
    const int idx_ref_before_bump = Gmap.idx_ref_pose_slam;
    Gmap.idx_ref_pose_slam = Gmap.KeyFDATA.size()-2; // update index reference for the next close loop

    //---------- fuse the points for closing the loop

    // Fuse_map_points(idx_kf_matched);

    // G6b: home-tag Step2 may leave idx_kf_matched == -1 when Get_matches
    // found no visual loop. Pose_SLAM already printed; do not index garbage.
    const auto match_dec = globalslam::g6b_cl::resolve_matched_kf_index(
      idx_kf_matched, idx_ref_before_bump, Gmap.KeyFDATA.size());
    if (!match_dec.valid) {
      // G6b.2: without a corresponding visual match, keep Pose_SLAM map
      // update only — do not invent a matched KF from the pose-graph gauge.
      cout << "cloop-> skip match/BA refinement after Pose_SLAM: "
           << match_dec.reason
           << " (visual_idx=" << idx_kf_matched
           << " ref=" << idx_ref_before_bump << ")" << endl;
    } else {
      int idx_matched_resolved = match_dec.index;

      // ----------- try to get new matches
      std::vector<int> idx_vl_new_kf;
      std::vector<int> idx_vl_old_kf;
      Update_gmap_with_new_matches(idx_matched_resolved,idx_vl_new_kf,idx_vl_old_kf);


      // ----------- refine the global map after loop with (local) bundle adjustment
      // G6d: post-loop BA refine shares Enable_global_ba with incremental BA.
      // LC-CONSISTENCY-A3: alignment δ_L is computed AFTER this refine.
      if (PAR.Enable_global_ba) {
        Update_gmap_with_BA(idx_vl_new_kf, idx_vl_old_kf);
        last_match_ba_refine_ran_ = true;
      } else {
        cout << "cloop-> skip post-loop BA refine (Enable_global_ba=false); "
             << "Pose_SLAM commit retained" << endl;
      }
    }

    // Final tip after Pose_SLAM (+ optional BA). Export Local alignment δ_L,
    // not incremental Pose_SLAM tip δ_G (A1 H1 / A2 design).
    const arma::vec::fixed<3> p_g_final = Gmap.KeyFDATA.back().t_c2n;
    Delta_kf_n = gmap_visual_sync::align_delta_xy(p_g_final, p_l_original);
    last_committed_camera_ = p_g_final;
    last_pose_slam_committed_ = true;
    last_sync_p_l_original_ = p_l_original;
    last_sync_keyframe_id_ = sync_kf_id;

    cout << "cloop-> SYNC_REQUEST align_xy=[" << Delta_kf_n[0] << " "
         << Delta_kf_n[1] << "] p_L_orig=[" << p_l_original[0] << " "
         << p_l_original[1] << "] p_G_final=[" << p_g_final[0] << " "
         << p_g_final[1] << "] kf_id=" << sync_kf_id
         << " ba_refine=" << (last_match_ba_refine_ran_ ? 1 : 0) << endl;

}
