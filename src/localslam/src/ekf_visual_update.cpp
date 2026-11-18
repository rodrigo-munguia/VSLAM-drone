#include "ekf.hpp"
#include "range_frame_association.hpp"
#include "visual_profile.hpp"




//void Jac_e_init_feat_func(cv::Point2d uvd, arma::vec& x, CAM &camera_parameters, double depth,arma::mat::fixed<3,13>& dy_dx, arma::mat::fixed<3,3>& dy_duvr);

// Visual update for euler-robot state vector
void EKF::Visual_update_e(FRAME *frame)
{
   using clock = std::chrono::steady_clock;
   const bool prof = localslam::visual_profile::enabled();
   const auto t_all = clock::now();
   auto t0 = t_all;

   Visual_delete_feats(); // delete old/bad visual features
   const double delete_ms = prof ? localslam::visual_profile::ms_since(t0) : 0.0;

  // cout << "frame->range: " << frame->range << endl;

   // B2: Range_init_min_m (sonar default 0.5) is exclusive: r > Range_init_min_m.
   t0 = clock::now();
   if(localslam::range_permits_undelayed_init(frame->range, PAR.Range_init_min_m) &&
      PAR.Visual_init_feats_method == "undelayed_range")
   {
      double v = sqrt(x(10)*x(10) + x(11)*x(11)); // current velocity over the x-y plane    

      if (v > PAR.Visual_update_min_vel_for_update_feats)
      { 
         Visual_update_e_init_feat_wr(frame); // Initialize new visual features into the system state
      }
   }
   if (PAR.Visual_init_feats_method == "delayed")
   {
      Visual_update_e_init_feat_delayed(frame);
   }   

   if(PAR.Visual_update_use_anchors == true)
   {
      Visual_update_e_init_anchors(); // Initialize anchors
   }
   const double init_ms = prof ? localslam::visual_profile::ms_since(t0) : 0.0;
   
    t0 = clock::now();
    int n_matches = Visual_match_feats(frame); // frame-to-frame visual matching    
    const double match_ms = prof ? localslam::visual_profile::ms_since(t0) : 0.0;

   int accepted = 0;
   t0 = clock::now();
   if (n_matches > 3) // if a minimun number of visual matches are found
   {
      accepted = 1;
      if(PAR.Visual_update_use_1RANSAC_for_validation == true)
      {
         //Visual_update_WO_val(); // tmp
         Visual_update_With_val_e(); // update system state with outlier rejection method
      }
      else
      {   
         Visual_update_WO_val(); // update system state without outlier rejection method
      } 
   }
   const double correct_ms = prof ? localslam::visual_profile::ms_since(t0) : 0.0;
   if (prof) {
     const int img_w = (frame != nullptr && !frame->image.empty()) ? frame->image.cols : 0;
     const int img_h = (frame != nullptr && !frame->image.empty()) ? frame->image.rows : 0;
     localslam::visual_profile::write_stage_row(
       delete_ms, init_ms, match_ms, correct_ms,
       localslam::visual_profile::ms_since(t_all), n_matches, accepted,
       static_cast<int>(FeatsDATA.size()), static_cast<int>(AnchorsDATA.size()),
       static_cast<int>(x.size()), img_w, img_h);
   }

}
