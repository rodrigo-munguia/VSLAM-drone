#ifndef GLOOP_H
#define GLOOP_H

#include <armadillo>
#include "parameters.hpp"
#include "../../common/Vision/vision.hpp"
#include "globalslam_types.hpp"
#include "g6b_home_tag_geometry.hpp"
#include "g6b_home_tag_capture.hpp"
#include "cloop_visual_retrieve.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <vector>

class CLOOP
{

    public:        
                
                   
        
        CLOOP(parameters &par) // constructor
        {
           PAR = par;
           CLOOP_init();
           cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        }
        
         

    private:
        
        

        GLOBAL_MAP Gmap; // Global map structure

        parameters PAR; // parameters structure
        
        CAM cam_parameters;

        arma::vec::fixed<3> Delta_kf_n;

        bool Close_loop;
        /** G6d: Pose_SLAM deltas applied to shared map (not merely attempted). */
        bool last_pose_slam_committed_{false};
        /** "tag_loop" | "visual_loop" for the latest successful commit. */
        std::string last_loop_source_;
        arma::vec::fixed<3> last_committed_camera_;
        bool last_match_ba_refine_ran_{false};

        double travel_dis;
        int n_cloop_intents;
        int n_kf_home;
        bool activate_CL_flag;
        /** G6b.3: epoch-owned capture session + frozen spatial reference. */
        globalslam::g6b_home_tag::HomeTagCaptureSession home_tag_capture_;

        arma::vec::fixed<3> last_pos;

        /** Detect intended home-tag on a KEYFRAME image; fills tvec / d_px / id. */
        bool Detect_home_tag_on_kf(
          const KEYFRAME & kf,
          int & tag_id,
          double tvec_out[3],
          double & d_px) const;

        std::vector<int> Get_n_not_visually_linked_kf(int idx_kf , int idx_kf_sup_limit );
        std::vector<int> Get_n_visually_linked_kf(int idx_kf, int min_strenght);

        void Get_kf_list_from_uncertainty(KEYFRAME &kf_cl,std::vector<int> &idx_nvl_kf);

        /** A2d/A3g: enumerate+match+rank historical visual candidates (no home/unc).
         *  apply_hist_travel_gate: visual Step only (tag Get_matches passes false). */
        std::vector<globalslam::cloop_visual_retrieve::RankedCandidate>
        Collect_ranked_visual_candidates(
          KEYFRAME & kf_cl, double * t_search_s, int * n_eligible_out = nullptr,
          int * n_hist_rejected_out = nullptr,
          bool apply_hist_travel_gate = true);
       
        void Get_matches(KEYFRAME &kf_cl, std::vector<int> &idx_pt_matches, std::vector<cv::Point2d> &image_points ,int &idx_kf_matched);
        
        bool Get_pos_of_current_kf(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl, std::vector<int> &idx_pt_matches, std::vector<cv::Point2d> &image_points, int step);
        bool Get_pos_of_current_kf_from_Home_tag(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl);


        void Update_gmap(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl,int &idx_kf_matched);
        
        void Pose_SLAM(arma::vec::fixed<3> &pos, std::vector<arma::vec::fixed<3>> &delta_kf_pos, std::vector<int> &idx_kf_opt);
        
        void Match_Gmap_points_into_KF(int idx_KF);
        void Update_Visibility_Graph_with_new_KF(int idx_KF);

        void Update_Visibility_Graph_A_B_links(std::vector<int> &idx_vl_kf_A,std::vector<int> &idx_vl_kf_B);

        void Fuse_map_points(int &idx_kf_matched);

        void Update_gmap_with_new_matches(int &idx_kf_matched, std::vector<int> &idx_vl_new_kf, std::vector<int> &idx_vl_old_kf);        
        void Match_list_of_points_into_KF(std::vector<int64> &idx_points, int idx_KF, std::vector<int64> &idx_matched_points);
        void Update_Visibility_Graph_with_matched_points_into_KF(std::vector<int64> &idx_points, int idx_KF);

        void Match_a_points_with_b_points(std::vector<int64> &idx_a_points,std::vector<int64> &idx_b_points,std::vector<int64> &idx_matched_a_pts,std::vector<int64> &idx_matched_b_pts);    

        void Update_gmap_with_BA(std::vector<int> &idx_vl_new_kf, std::vector<int> &idx_vl_old_kf); 

    public:

        void CLOOP_init();
        
        void Set_GlobalMap(GLOBAL_MAP &Gmap_c);

        void Get_GlobalMap(GLOBAL_MAP &Gmap_c);

        void Step(KEYFRAME &kf_cl);
        void Step2(KEYFRAME &kf_cl);

        arma::vec::fixed<3>  Get_delta_pos();

        bool Get_close_loop_state();
        void Set_close_loop_state(bool state);

        /** True only when Pose_SLAM produced and applied non-empty deltas. */
        bool Last_loop_committed() const { return last_pose_slam_committed_; }
        const std::string & Last_loop_source() const { return last_loop_source_; }
        arma::vec::fixed<3> Last_committed_camera() const
        {
          return last_committed_camera_;
        }
        bool Last_match_ba_refine_ran() const { return last_match_ba_refine_ran_; }

        /** Stamp n_kf_home on an already-ready/lifecycle ref (legacy hook). */
        void Set_n_kf_rechome(
          const slam_identity::UuidBytes & session_id, uint64_t epoch_id);

        /** G6b.3 RecHome capture: begin / offer LS snapshot / finalize. */
        bool Begin_home_tag_capture(
          const slam_identity::UuidBytes & session_id,
          uint64_t epoch_id,
          std::string & reason);
        void Request_home_tag_capture_finalize();
        bool Try_finalize_home_tag_capture();
        void Offer_home_tag_capture_kf(const KEYFRAME & kf);
        void Invalidate_home_tag_capture(const std::string & reason);
        bool Home_tag_capture_is_recording() const;
        bool Home_tag_spatial_ready() const;
        const globalslam::g6b_home_tag::HomeTagReference & Home_tag_reference() const;

        void Set_Activate_Deactivate_CL(bool value);

        STOREC store;

    
};


#endif


