#include "cloop.hpp"

#include <chrono>

//-------------------------------------------
// Init Cloop
void CLOOP::CLOOP_init()
{ 
  Gmap.idx_ref_pose_slam = 0;
  Gmap.AnchorsDATA.clear();
  Gmap.KeyFDATA.clear();
  Gmap.Vgraph.clear();

  cam_parameters.distortions = &PAR.Mono_cam_distortions[0];
  cam_parameters.cc[0] = PAR.Mono_cam_cc_u;
  cam_parameters.cc[1] = PAR.Mono_cam_cc_v;
  cam_parameters.fc[0] = PAR.Mono_cam_fc_u;
  cam_parameters.fc[1] = PAR.Mono_cam_fc_v;
  cam_parameters.alpha_c = PAR.Mono_cam_alpha_c;

  Close_loop = false;

  travel_dis = 0;
  n_cloop_intents = 0;
  last_pos = {0,0,0};

  n_kf_home = 0;
  activate_CL_flag = false;
  home_tag_capture_.clear();
  {
    globalslam::g6b_home_tag::CaptureParams cp;
    cp.home_tag_id = PAR.CL_home_tag_id;
    cp.tag_size_m = PAR.CL_home_tag_size_m;
    cp.min_inliers = PAR.CL_home_capture_min_inliers;
    cp.min_inlier_fraction = PAR.CL_home_capture_min_inlier_fraction;
    cp.max_spread_m = PAR.CL_home_capture_max_spread_m;
    cp.max_samples = PAR.CL_home_capture_max_samples;
    cp.gate_px = static_cast<double>(PAR.CL_tag_max_xy_dis_px);
    home_tag_capture_.set_params(cp);
  }
           
  // initialice store structure for log system statistics    
  store.time_search_per_step.first.clear();
  store.time_search_per_step.second.clear();
  store.cloop_stats.clear();

  cout << "cloop -> initialized" << endl;  

}

//------------------------------------------
// Set global map
// Rodrigo M. 20222
void CLOOP::Set_GlobalMap(GLOBAL_MAP &Gmap_c)
{

  Gmap = Gmap_c;

}
//------------------------------------------
// Set global map
// Rodrigo M. 20222
void CLOOP::Get_GlobalMap(GLOBAL_MAP &Gmap_c)
{
  Gmap_c = Gmap;
}

//------------------------------------------
arma::vec::fixed<3>  CLOOP::Get_delta_pos()
{
  return Delta_kf_n;      
}
//----------------------------------------
bool CLOOP::Get_close_loop_state()
{
  return Close_loop;
}
void CLOOP::Set_close_loop_state(bool state)
{
  Close_loop = state;
}
void CLOOP::Set_n_kf_rechome(
  const slam_identity::UuidBytes & session_id, uint64_t epoch_id)
{
  n_kf_home = static_cast<int>(Gmap.KeyFDATA.size());
  // Finalize path stamps n_kf_home onto the capture reference.
  (void)session_id;
  (void)epoch_id;
  cout << "Cloop-> Set number of KF belonging to home area to: " << n_kf_home
       << endl;
}
bool CLOOP::Begin_home_tag_capture(
  const slam_identity::UuidBytes & session_id,
  uint64_t epoch_id,
  std::string & reason)
{
  globalslam::g6b_home_tag::CaptureParams cp;
  cp.home_tag_id = PAR.CL_home_tag_id;
  cp.tag_size_m = PAR.CL_home_tag_size_m;
  cp.min_inliers = PAR.CL_home_capture_min_inliers;
  cp.min_inlier_fraction = PAR.CL_home_capture_min_inlier_fraction;
  cp.max_spread_m = PAR.CL_home_capture_max_spread_m;
  cp.max_samples = PAR.CL_home_capture_max_samples;
  cp.gate_px = static_cast<double>(PAR.CL_tag_max_xy_dis_px);
  home_tag_capture_.set_params(cp);
  const bool ok = home_tag_capture_.begin(session_id, epoch_id, reason);
  if (ok) {
    cout << "Cloop-> home-tag capture STARTED (tag_id=" << PAR.CL_home_tag_id
         << " epoch=" << epoch_id << ")" << endl;
  } else {
    cout << "Cloop-> home-tag capture begin refused: " << reason << endl;
  }
  return ok;
}
void CLOOP::Request_home_tag_capture_finalize()
{
  home_tag_capture_.request_finalize();
  n_kf_home = static_cast<int>(Gmap.KeyFDATA.size());
  cout << "Cloop-> home-tag capture finalize requested "
       << "(n_kf_home=" << n_kf_home
       << " cutoff_kf=" << home_tag_capture_.cutoff_keyframe_id()
       << " samples=" << home_tag_capture_.sample_count() << ")" << endl;
}
bool CLOOP::Try_finalize_home_tag_capture()
{
  if (!home_tag_capture_.finalize_requested() &&
    home_tag_capture_.state() != globalslam::g6b_home_tag::CaptureState::Recording)
  {
    return home_tag_capture_.is_ready();
  }
  if (!home_tag_capture_.finalize_requested()) {
    return false;
  }
  const bool ok = home_tag_capture_.finalize(n_kf_home);
  const auto & ref = home_tag_capture_.reference();
  const auto & diag = home_tag_capture_.diagnostics();
  if (ok) {
    cout << "Cloop-> home-tag capture READY p_H_N=["
         << ref.p_H_N[0] << " " << ref.p_H_N[1] << " " << ref.p_H_N[2]
         << "] inliers=" << ref.n_inliers << "/" << ref.n_candidates
         << " spread_m=" << ref.spread_m
         << " n_kf_home=" << ref.n_kf_home << endl;
  } else {
    cout << "Cloop-> home-tag capture FAILED: " << diag.fail_reason
         << " (candidates=" << diag.n_candidates
         << " inliers=" << diag.n_inliers << ")" << endl;
  }
  return ok;
}
void CLOOP::Offer_home_tag_capture_kf(const KEYFRAME & kf)
{
  if (!home_tag_capture_.is_recording()) {
    return;
  }
  home_tag_capture_.note_keyframe_seen(
    kf.keyframe_id, kf.source_time_ms, kf.source_stamp_valid);

  int tag_id = -1;
  double tvec[3]{0, 0, 0};
  double d_px = 0;
  if (!Detect_home_tag_on_kf(kf, tag_id, tvec, d_px)) {
    return;
  }

  using globalslam::g6b_home_tag::RecordingObservation;
  using globalslam::g6b_home_tag::flatten_Rn2c_row_major;
  using globalslam::g6b_home_tag::tag_position_from_camera;

  RecordingObservation obs;
  obs.session_id = kf.session_id;
  obs.epoch_id = kf.epoch_id;
  obs.keyframe_id = kf.keyframe_id;
  obs.source_time_ms = kf.source_time_ms;
  obs.source_stamp_valid = kf.source_stamp_valid;
  obs.tag_id = tag_id;
  obs.t_c2n[0] = kf.t_c2n[0];
  obs.t_c2n[1] = kf.t_c2n[1];
  obs.t_c2n[2] = kf.t_c2n[2];
  flatten_Rn2c_row_major(
    kf.Rn2c(0, 0), kf.Rn2c(0, 1), kf.Rn2c(0, 2),
    kf.Rn2c(1, 0), kf.Rn2c(1, 1), kf.Rn2c(1, 2),
    kf.Rn2c(2, 0), kf.Rn2c(2, 1), kf.Rn2c(2, 2),
    obs.Rn2c_rm);
  obs.tvec[0] = tvec[0];
  obs.tvec[1] = tvec[1];
  obs.tvec[2] = tvec[2];
  obs.d_px = d_px;
  // Original LS camera pose only — never Gmap.KeyFDATA optimized poses.
  tag_position_from_camera(
    obs.t_c2n[0], obs.t_c2n[1], obs.t_c2n[2],
    obs.tvec[0], obs.tvec[1], obs.tvec[2],
    obs.Rn2c_rm,
    obs.p_H_N[0], obs.p_H_N[1], obs.p_H_N[2]);

  std::string reject;
  if (home_tag_capture_.offer(obs, reject)) {
    cout << "Cloop-> home-tag capture sample kf=" << kf.keyframe_id
         << " p_H_N=[" << obs.p_H_N[0] << " " << obs.p_H_N[1] << " "
         << obs.p_H_N[2] << "] n=" << home_tag_capture_.sample_count() << endl;
  } else if (reject != "duplicate_keyframe_id" && reject != "after_cutoff_id" &&
    reject != "after_cutoff_time")
  {
    cout << "Cloop-> home-tag capture reject kf=" << kf.keyframe_id
         << ": " << reject << endl;
  }
}
void CLOOP::Invalidate_home_tag_capture(const std::string & reason)
{
  if (home_tag_capture_.is_recording()) {
    cout << "Cloop-> home-tag capture INVALIDATED: " << reason << endl;
    home_tag_capture_.fail(reason);
  }
}
bool CLOOP::Home_tag_capture_is_recording() const
{
  return home_tag_capture_.is_recording();
}
bool CLOOP::Home_tag_spatial_ready() const
{
  return home_tag_capture_.is_ready();
}
const globalslam::g6b_home_tag::HomeTagReference & CLOOP::Home_tag_reference() const
{
  return home_tag_capture_.reference();
}
void CLOOP::Set_Activate_Deactivate_CL(bool value)
{
  activate_CL_flag = value;
}


//------------------------------------------
// Visual loop Step (A2d): autonomous commit (no 'c'/GoHome), ranked historical
// retrieval, bounded PnP fallback. Tag Step2 activation unchanged.
void CLOOP::Step(KEYFRAME &kf_cl)
{
    static double total_comp_time = 0;
    static double step = 0;
    step++;
    last_pose_slam_committed_ = false;
    last_match_ba_refine_ran_ = false;

    static int n_kf_wo_cl = 0;
    n_kf_wo_cl++;

    using globalslam::cloop_visual_retrieve::AttemptDiag;
    using globalslam::cloop_visual_retrieve::compute_significance_metrics;
    using globalslam::cloop_visual_retrieve::max_attempts_clamped;
    using globalslam::cloop_visual_retrieve::positions_with_current_endpoint;

    AttemptDiag diag;
    diag.current_kf_idx =
      Gmap.KeyFDATA.empty() ? -1 : static_cast<int>(Gmap.KeyFDATA.size()) - 1;

    double t_search = 0;
    int n_eligible = 0;
    int n_hist_rejected = 0;
    auto ranked = Collect_ranked_visual_candidates(
      kf_cl, &t_search, &n_eligible, &n_hist_rejected,
      /*apply_hist_travel_gate=*/true);
    diag.t_search_s = t_search;
    diag.n_eligible = n_eligible;
    diag.n_hist_rejected = n_hist_rejected;
    diag.n_above_threshold = static_cast<int>(ranked.size());
    total_comp_time += t_search;

    double last_dis = arma::as_scalar(arma::norm(kf_cl.t_c2n - last_pos));
    if (last_dis > 2) {
      travel_dis += last_dis;
      last_pos = kf_cl.t_c2n;
    }

    // A2d: visual path commits without sticky activate_CL_flag / GoHome/'c'.
    // Step is only dispatched when Enable_visual_loop_closure is true (G6d).
    if (!ranked.empty() && travel_dis > 10) {
      const int n_try =
        max_attempts_clamped(PAR.CL_visual_max_pnp_attempts,
          static_cast<int>(ranked.size()));
      cout << "cloop-> visual LC attempt: cur_kf=" << diag.current_kf_idx
           << " eligible=" << diag.n_eligible
           << " hist_rej=" << diag.n_hist_rejected
           << " above_thr=" << diag.n_above_threshold
           << " travel_dis=" << travel_dis
           << " min_hist_m=" << PAR.CL_min_historical_travel_m
           << " search_s=" << diag.t_search_s << " top=";
      for (int i = 0; i < n_try; ++i) {
        cout << ranked[static_cast<size_t>(i)].idx_kf << ":"
             << ranked[static_cast<size_t>(i)].n_matches
             << (i + 1 < n_try ? "," : "");
      }
      cout << endl;

      // Significance uses mapped centres + unappended current CL endpoint.
      std::vector<std::array<double, 3>> mapped;
      mapped.reserve(Gmap.KeyFDATA.size());
      for (const auto & kf : Gmap.KeyFDATA) {
        mapped.push_back({kf.t_c2n[0], kf.t_c2n[1], kf.t_c2n[2]});
      }
      const std::array<double, 3> cur_pose =
        {kf_cl.t_c2n[0], kf_cl.t_c2n[1], kf_cl.t_c2n[2]};
      const auto kf_centres = positions_with_current_endpoint(
        mapped, cur_pose, /*append_current_endpoint=*/true, nullptr);
      const int idx_endpoint =
        static_cast<int>(kf_centres.size()) - 1;
      const int ba_limit =
        std::max(0, static_cast<int>(Gmap.KeyFDATA.size()) -
          PAR.BA_max_n_kf_optimized);

      const auto t_pnp0 = std::chrono::high_resolution_clock::now();

      for (int i = 0; i < n_try; ++i) {
        auto & cand = ranked[static_cast<size_t>(i)];
        diag.pnp_attempts++;
        diag.selected_kf_idx = cand.idx_kf;
        diag.selected_n_matches = cand.n_matches;

        const auto sig = compute_significance_metrics(
          kf_centres, idx_endpoint, cand.idx_kf);
        diag.last_hist_path_m = sig.hist_path_m;

        arma::vec::fixed<3> kf_cl_pos;
        const bool get_pos = Get_pos_of_current_kf(
          kf_cl_pos, kf_cl, cand.idx_pt_matches, cand.image_points, step);

        if (!get_pos) {
          diag.last_pnp_reason = "pnp_or_gate_reject";
          cout << "cloop-> visual LC: PnP reject try=" << diag.pnp_attempts
               << " matched_kf=" << cand.idx_kf
               << " matches=" << cand.n_matches
               << " idx_sep=" << sig.idx_sep
               << " hist_path_m=" << sig.hist_path_m
               << " euclid_m=" << sig.euclid_m << endl;
          continue;
        }

        diag.accepted = true;
        diag.last_pnp_reason = "ok";
        last_loop_source_ = "visual_loop";
        const int ref_before = Gmap.idx_ref_pose_slam;
        Update_gmap(kf_cl_pos, kf_cl, cand.idx_kf);
        diag.committed = last_pose_slam_committed_;
        cout << "cloop-> visual LC: matched_kf=" << cand.idx_kf
             << " matches=" << cand.n_matches
             << " pnp_try=" << diag.pnp_attempts
             << "/" << n_try
             << " pos=[" << kf_cl_pos[0] << " " << kf_cl_pos[1] << " "
             << kf_cl_pos[2] << "]"
             << " ref=" << ref_before
             << " committed=" << (diag.committed ? 1 : 0)
             << " travel_dis=" << travel_dis
             << " hist_path_m=" << sig.hist_path_m << endl;
        cout << "cloop-> visual LC significance: cur_kf="
             << diag.current_kf_idx
             << " matched_kf=" << cand.idx_kf
             << " idx_sep=" << sig.idx_sep
             << " hist_path_m=" << sig.hist_path_m
             << " euclid_m=" << sig.euclid_m
             << " ba_limit=" << ba_limit
             << " hist_rej=" << diag.n_hist_rejected
             << " travel_dis=" << travel_dis << endl;

        if (PAR.Stats && !store.cloop_stats.empty()) {
          store.cloop_stats.back().distance_traveled = travel_dis;
        }

        travel_dis = 0;
        n_cloop_intents = 0;
        n_kf_wo_cl = 0;
        Close_loop = last_pose_slam_committed_;
        break;
      }

      const auto t_pnp1 = std::chrono::high_resolution_clock::now();
      diag.t_pnp_s =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t_pnp1 - t_pnp0)
          .count() *
        1e-9;
      total_comp_time += diag.t_pnp_s;

      if (!diag.accepted && diag.pnp_attempts > 0) {
        cout << "cloop-> visual LC: no PnP accept after "
             << diag.pnp_attempts << " tries (eligible=" << diag.n_eligible
             << " above_thr=" << diag.n_above_threshold
             << " search_s=" << diag.t_search_s
             << " pnp_s=" << diag.t_pnp_s << ")" << endl;
      }
    }

    if (PAR.Stats) {
      store.total_comp_time = total_comp_time;
      store.time_search_per_step.first.push_back(step);
      store.time_search_per_step.second.push_back(t_search);
    }
}


//------------------------------------------
// Perform a full step for trying to close a loop
// Rodrigo M. 20222
void CLOOP::Step2(KEYFRAME &kf_cl)
{
    static double total_comp_time = 0;
    static double step = 0;
    step++;
    last_pose_slam_committed_ = false;
    last_match_ba_refine_ran_ = false;
    
    static int n_kf_wo_cl = 0;
    // Get matches from old kf
    std::vector<int> idx_pt_matches;
    std::vector<cv::Point2d> image_points;
    // G6b: home-tag Step2 does not require visual matches, so Get_matches may
    // never assign idx_kf_matched. Uninitialized use after Pose_SLAM → SEGV.
    int idx_kf_matched = -1;
    
      auto ct_i = std::chrono::high_resolution_clock::now();  // take computation time

    Get_matches(kf_cl,idx_pt_matches,image_points,idx_kf_matched);
    n_kf_wo_cl++;

      auto ct_f = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_f - ct_i);
      double comp_search_time_per_step = elapsed.count() * 1e-9;
      total_comp_time = total_comp_time + comp_search_time_per_step;

    double last_dis = arma::as_scalar(arma::norm(kf_cl.t_c2n - last_pos));
    if(last_dis > 2)
    {
      travel_dis += last_dis;
      last_pos = kf_cl.t_c2n;
    }          
     
    //cout << idx_pt_matches.size() << endl; 
    //if (idx_pt_matches.size() > PAR.CL_min_n_matches && travel_dis > 10)
    if (travel_dis > 10)
    { 
      bool get_pos;  
      arma::vec::fixed<3> kf_cl_pos;
      
      //get_pos = Get_pos_of_current_kf(kf_cl_pos,kf_cl, idx_pt_matches, image_points,step);
      get_pos = Get_pos_of_current_kf_from_Home_tag(kf_cl_pos,kf_cl);


      if(get_pos == true && activate_CL_flag == true)
      {
        last_loop_source_ = "tag_loop";
        Update_gmap(kf_cl_pos,kf_cl,idx_kf_matched);
        cout << "Distance traveled since origin/last pos update: " << travel_dis << endl;
        
        if(PAR.Stats && !store.cloop_stats.empty()){
          store.cloop_stats.back().distance_traveled = travel_dis;
        }          
        
        travel_dis = 0;
        n_cloop_intents = 0;
        n_kf_wo_cl = 0;
        Close_loop = last_pose_slam_committed_;
      }

    }

    // store statitics 
     if(PAR.Stats){
          //cout << "Cloop step:" << step << endl;
          store.total_comp_time = total_comp_time;
          store.time_search_per_step.first.push_back(step);
          store.time_search_per_step.second.push_back(comp_search_time_per_step);          
     }           

}
