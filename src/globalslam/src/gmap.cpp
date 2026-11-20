#include "gmap.hpp"
#include "gmap_ba_d5_log.hpp"
#include "gmap_relative_kf_init.hpp"


//-------------------------------------------
// Init Gmap
void GMAP::InitGmap()
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
           
    total_init_anchors = 0;
    total_delete_anchors = 0;
    birth_gate_attempts = 0;
    birth_gate_accepted = 0;
    birth_gate_rejected = 0;
    have_prev_ekf_cam_pose_ = false;

    // initialice store structure for log system statistics            
    store.n_init_anchors = 0;            
    store.n_delete_anchors = 0;            
    store.total_comp_time = 0;            
    store.n_anchors_per_step.first.clear();
    store.n_anchors_per_step.second.clear();
    store.time_per_step.first.clear(); 
    store.time_per_step.second.clear(); 
    store.n_kf_per_step.first.clear();
    store.n_kf_per_step.second.clear(); 

    cout << "gmap -> initialized" << endl; 
}


//------------------------------------------
// Add a Keyframe to the buffer
// Rodrigo M. 20222
void GMAP::Add_Kf(KEYFRAME &kf)
{

    Kf_buffer.push_back(kf);
    //cout << "Kf " << Kf_buffer.size() << " added" << endl;

}

//----------------------------------------
// Get Global Map
void GMAP::Get_GlobalMap(GLOBAL_MAP &Gmap_c)
{
   Gmap_c = Gmap;
}
//------------------------------------------
// Set global map
// Rodrigo M. 20222
void GMAP::Set_GlobalMap(GLOBAL_MAP &Gmap_c)
{
  Gmap = Gmap_c;
}

//------------------------------------------
arma::vec::fixed<3>  GMAP::Get_delta_pos()
{
  return Delta_kf_n;      
}


//------------------------------------------
// Perform a full global map update step
// Rodrigo M. 20222
void GMAP::Update()
{
    static double total_comp_time = 0;
    static double step = 0;
    step++;
    //------------------------------------------------------
    // After a close loop perform a full update of the visibility graph
    
    
    //-------------------------------------------------------
    // Grow the global map: Add new Keyframes and new Map Point
    auto ct_i = std::chrono::high_resolution_clock::now();  
    bool new_kf = false;
    for(int k = 0; k < Kf_buffer.size() ; k++ )
    {   
        // Add new keyframes to the global map
        Gmap.KeyFDATA.push_back(Kf_buffer[k]);
        new_kf = true; // if a new keyframe has been added set this flag
        KEYFRAME & kf_new = Gmap.KeyFDATA.back();
        const int idx_new = static_cast<int>(Gmap.KeyFDATA.size()) - 1;

        // Absolute EKF camera pose as admitted (before any relative rewrite).
        const gmap_relative_kf_init::CamPose T_ekf_curr =
          gmap_relative_kf_init::from_keyframe(kf_new);
        gmap_ba_d5::dump_kf_pose("ekf_admit", idx_new, kf_new);

        // BA-D6 / CONFIG-A1 / KF-Z-A5: hybrid XY-rel/Z-abs is production default.
        // Must run before match / triangulation so the pipeline uses T_GS.
        // Absolute admit: Init_relative_kf_pose_enable:=false / env=0.
        // Full-relative: Init_hybrid_kf_z_absolute_enable:=false / HYBRID env=0.
        const gmap_relative_kf_init::InitMode init_mode =
          gmap_relative_kf_init::select_init_mode(
            PAR.Init_relative_kf_pose_enable,
            PAR.Init_hybrid_kf_z_absolute_enable,
            have_prev_ekf_cam_pose_,
            idx_new);
        if (init_mode != gmap_relative_kf_init::InitMode::Absolute)
        {
          const gmap_relative_kf_init::CamPose T_gs_prev =
            gmap_relative_kf_init::from_keyframe(Gmap.KeyFDATA[idx_new - 1]);
          const gmap_relative_kf_init::CamPose T_full_rel =
            gmap_relative_kf_init::propagate(
              T_gs_prev, prev_ekf_cam_pose_, T_ekf_curr);
          gmap_relative_kf_init::CamPose T_gs_curr = T_full_rel;
          if (init_mode == gmap_relative_kf_init::InitMode::HybridXyRelZAbs) {
            T_gs_curr = gmap_relative_kf_init::propagate_xy_rel_z_abs(
              T_gs_prev, prev_ekf_cam_pose_, T_ekf_curr);
          }
          gmap_relative_kf_init::apply_cam_pose(kf_new, T_gs_curr);
          gmap_ba_d5::dump_kf_pose(
            init_mode == gmap_relative_kf_init::InitMode::HybridXyRelZAbs
              ? "hybrid_admit"
              : "rel_admit",
            idx_new, kf_new);
          gmap_relative_kf_init::maybe_log_admit_z(
            init_mode,
            idx_new,
            kf_new.keyframe_id,
            T_ekf_curr.t_c2n(2),
            T_full_rel.t_c2n(2),
            kf_new.t_c2n(2),
            T_gs_prev.t_c2n(2));
        } else {
          gmap_relative_kf_init::maybe_log_admit_z(
            init_mode,
            idx_new,
            kf_new.keyframe_id,
            T_ekf_curr.t_c2n(2),
            T_ekf_curr.t_c2n(2),
            kf_new.t_c2n(2),
            have_prev_ekf_cam_pose_ && idx_new > 0
              ? Gmap.KeyFDATA[idx_new - 1].t_c2n(2)
              : T_ekf_curr.t_c2n(2));
        }

        prev_ekf_cam_pose_ = T_ekf_curr;
        have_prev_ekf_cam_pose_ = true;
        //cout << Kf_buffer[k].frame << endl;
        
            //------------- Try to match the global map points in the new kf  
            Match_Gmap_points_into_new_KF();
            
            //------------- Update the visibility graph for taking into account the new keyframe
            
            Update_Visibility_Graph_with_new_KF();
           
         
        if (Gmap.KeyFDATA.size() > 1)        
        { 
            //------------- Initialize new global map points
            std::vector<int64> idx_new_points = Init_new_map_points();
            //cout << idx_new_points.size() << endl;
            //cout << Gmap.Vgraph << endl;
            
            if (idx_new_points.size() > 0) // if new map points have been initialized
            {          
                std::vector<int> idx_kf_m; 
                std::vector<int64> idx_pt_m; 
                //-------------- Try to match the new map points in previous KF
                Match_New_points_into_previous_KF(idx_new_points, idx_kf_m, idx_pt_m );                

                //-------------- Update the visibility graph for taking into account the new map points
                //cout << " Before: " << endl;
                //cout << Gmap.Vgraph << endl;
                Update_Visibility_Graph_with_matches_of_new_points(idx_new_points,idx_kf_m, idx_pt_m );
                // cout << " After: " << endl;
                //cout << Gmap.Vgraph << endl;
                //( solve first row/column of zeros)

                //cout << Gmap.Vgraph << endl;       
            }
        }
        
    }
    Kf_buffer.clear();
    //--------------------------------------------------------

   if ((new_kf == true)&&(Gmap.KeyFDATA.size()>2))
        {
           // Delete map points that have not been matched in a minimun number of keyframes
           std::vector<std::vector<int>> kf_vg_info; 
           Delete_points(kf_vg_info);
           
           // Update visibility graph           
           Update_Visibility_Graph_with_deleted_points(kf_vg_info);
           
           //cout << Gmap.Vgraph << endl;
           
           // G6d: incremental BA is independently gated (feedback is separate).
           if (PAR.Enable_global_ba) {
             Bundle_adjustment();
           } else {
             Delta_kf_n.zeros();
           } 

           // Update matches and visibility graph  
           /*
           cout << endl;
           for (int i = 0; i < Gmap.Vgraph.n_rows ; i++ )
            {
                for (int j = 0; j < Gmap.Vgraph.n_rows ; j++ )
                    {
                        cout << (int)Gmap.Vgraph.at(i,j) << " ";
                    }
                    cout << endl;    
                }
           */     
           int q = 10;
        }

     auto ct_f = std::chrono::high_resolution_clock::now();
     auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_f - ct_i);
     double comp_time_per_step = elapsed.count() * 1e-9;   
     total_comp_time = total_comp_time + comp_time_per_step;

     // store statitics 
     if(PAR.Stats){
                //cout << "GSLAM step:" << step << endl;
                store.n_kf = Gmap.KeyFDATA.size();
                store.n_init_anchors = total_init_anchors;
                store.n_delete_anchors = total_delete_anchors;
                store.total_comp_time = total_comp_time;
                store.time_per_step.first.push_back(step);
                store.time_per_step.second.push_back(comp_time_per_step);
                store.n_anchors_per_step.first.push_back(step);
                store.n_anchors_per_step.second.push_back(total_init_anchors-total_delete_anchors);
                store.n_kf_per_step.first.push_back(step);
                store.n_kf_per_step.second.push_back(Gmap.KeyFDATA.size());                
                
      }   



}