#include "cloop.hpp"
#include "g6b_cl_match_index.hpp"


void CLOOP::Update_gmap(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl,int &idx_kf_matched)
{
     last_pose_slam_committed_ = false;
     last_match_ba_refine_ran_ = false;

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
     
    // update kf position
    for(int i = 0 ; i < idx_kf_opt.size() ; i++)
    {   
        int idx_kf = idx_kf_opt[i];
        Gmap.KeyFDATA[idx_kf].t_c2n += delta_kf_pos[i]; 

    }
    // Update Point position
    for(int i = Gmap.AnchorsDATA.size()-1; i >= 0; i--)
    {
        int idx_init_kf = Gmap.AnchorsDATA[i].init_KF;

        auto it = find(idx_kf_opt.begin(), idx_kf_opt.end(), idx_init_kf);
                
        if(it != idx_kf_opt.end())
        {  
          int index = it - idx_kf_opt.begin();
           Gmap.AnchorsDATA[i].AnchorState += delta_kf_pos[index]; 
        }

    }

    
    
    Delta_kf_n = delta_kf_pos[delta_kf_pos.size()-1];
    last_committed_camera_ = Gmap.KeyFDATA.back().t_c2n;
    last_pose_slam_committed_ = true;

    // cout << "cloop: delta pos " << Delta_kf_n.t() << endl; 

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
      return;
    }
    int idx_matched_resolved = match_dec.index;

    // ----------- try to get new matches
    std::vector<int> idx_vl_new_kf; 
    std::vector<int> idx_vl_old_kf;
    Update_gmap_with_new_matches(idx_matched_resolved,idx_vl_new_kf,idx_vl_old_kf);
    
    
    // ----------- refine the global map after loop with (local) bundle adjustment
    // G6d: post-loop BA refine shares Enable_global_ba with incremental BA.
    // Refinement is not a second BA feedback emission — loop Δ already covers it.
    if (PAR.Enable_global_ba) {
      Update_gmap_with_BA(idx_vl_new_kf, idx_vl_old_kf);
      last_match_ba_refine_ran_ = true;
    } else {
      cout << "cloop-> skip post-loop BA refine (Enable_global_ba=false); "
           << "Pose_SLAM commit retained" << endl;
    }

}  
