#include "ekf.hpp"




//void Jac_e_init_feat_func(cv::Point2d uvd, arma::vec& x, CAM &camera_parameters, double depth,arma::mat::fixed<3,13>& dy_dx, arma::mat::fixed<3,3>& dy_duvr);

// Visual update for euler-robot state vector
void EKF::Visual_update_e(FRAME *frame)
{
   
   Visual_delete_feats(); // delete old/bad visual features

  // cout << "frame->range: " << frame->range << endl;

   if(frame->range > .5 && PAR.Visual_init_feats_method == "undelayed_range")
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
   
    int n_matches = Visual_match_feats(frame); // frame-to-frame visual matching    

   if (n_matches > 3) // if a minimun number of visual matches are found
   {
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



}




