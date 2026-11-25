#ifndef EKF_H
#define EKF_H

//#include </usr/include/armadillo>
#include <armadillo>
#include "parameters.hpp"
#include "localslam_types.hpp"
#include "ekf_types.hpp"
#include <opencv2/opencv.hpp>
#include "../../common/keyframe_types.hpp"
#include "../../common/Vision/vision.hpp"



using namespace arma;

class EKF
{

    public:        
                
                   
        
        EKF(parameters &par) // constructor
        {
            PAR = par;             
            initialized = false;  

            // initialice store structure for log system statistics 
            store.n_init_feats = 0;
            store.n_init_anchors = 0;
            store.n_delete_feats = 0;
            store.n_delete_anchors = 0;
            store.total_exec_time = 0;
            store.total_comp_time = 0;
            store.n_feats_per_frame.first.clear();
            store.n_feats_per_frame.second.clear();
            store.n_anchors_per_frame.first.clear();
            store.n_anchors_per_frame.second.clear();
            store.time_per_frame.first.clear(); 
            store.time_per_frame.second.clear();                       
            
        }
        
         

    private:

        arma::vec x;  // State vector
        arma::mat P;  // Covariance Matrix 
        
        arma::mat::fixed<2,2> Pc; // Covariance Matrix for loop closure
        double Cl_radius_uncertainty;
        bool Compute_Pc_flag;
        

        std::vector<FEAT> FeatsDATA;
        std::vector<FEAT> AnchorsDATA;

        CAM cam_parameters;
        arma::vec::fixed<3> t_c2r;  // vector defining the position of the camara respect to the robot frame 
        arma::mat::fixed<3,3> Rr2c; // Robot to camera rotation matrix

        KEYFRAME KF_selected; // for storing selected keyframes        
        KEYFRAME KF_cl; // for the closing loop process        
        bool NewKF_available;
        bool New_KF_cl;

        bool NewRobotState_available;        

        bool initialized; 
        parameters PAR;
        void prediction(double delta_t);
        void prediction_quat_noise_driven(double delta_t);
        void prediction_euler_differential_robot(ODOD &odod);
        void prediction_euler_odometry_vw_robot(ODOV &odov);
        void prediction_euler_noise_driven(double delta_t);
        void prediction_compute_CL_cov(double delta_t);
        void init_quat_system_state(DATA &dat);
        void init_euler_system_statet(DATA &dat);
        void Attitude_quat_Update(ATT &att);
        void Attitude_euler_Update(ATT &att);
        void Speed_Update2(SPD &spd);
        void Speed_Update_nav_euler(SPD &spd);
        /** Returns true if altitude home was newly latched on this call. */
        bool Altitude_update(ALT &alt);
        void Rangetobase_update(RANGETOBASE &rangetobase);
        void Visual_update_e(FRAME *frame);
        void Visual_update_e_init_feat_wr(FRAME *frame);
        void Visual_update_e_init_feat_delayed(FRAME *frame);
        void Visual_update_e_init_anchors();
        void Get_img_points_for_init_feats(FRAME *frame, vector<cv::KeyPoint>& Points, cv::Mat &Descriptors, vector<cv::Mat> &Patches_init, vector<cv::Mat> &Patches_match);
        arma::vec::fixed<3> Triangulate_sigle_3d_point(cv::Point2f &uv1d,cv::Point2f &uv2d,arma::mat::fixed<3,3> &Rn2c_1,arma::vec::fixed<3> &t_c2n_1,arma::mat::fixed<3,3> &Rn2c_2,arma::vec::fixed<3> &t_c2n_2 );

        
        void Remove_close_points(vector<cv::KeyPoint>& output_KP, vector<cv::KeyPoint>&  input_KP);
        void Delete_i_feat_(int idx);
        int Visual_match_feats(FRAME *frame);
        int Visual_match_feats_by_descriptors_e(FRAME *frame);
        void Visual_match_SelectKeyFrames(FRAME *frame, int n_m_points,arma::vec::fixed<3> t_c2r, arma::mat::fixed<3,3> Rr2c, arma::vec::fixed<3> r_N, arma::mat::fixed<3,3> Rn2r);        
        void Visual_delete_feats();
        void Visual_update_WO_val();
        void Visual_update_With_val_e();
        void Visual_update_anchors_WO_val_e();
        void Visual_update_feats_WO_val_e();
        void Visual_update_One_Point_ransac_hypotheses_e(std::vector<int> &z_li_f,std::vector<int> &z_li_a);
        void Visual_update_One_Point_ransac_e(std::vector<int> &z_lx_f,std::vector<int> &z_lx_a);
        void Visual_update_One_Point_ransac_rescue_hi_inliers_e(std::vector<int> &z_li_f,std::vector<int> &z_li_a,std::vector<int> &z_hi_f,std::vector<int> &z_hi_a );
        
        void Store_info(double time);

        double yaw_at_home{0.0};

        /** Instance-owned altitude home (replaces process-static latch). */
        bool alt_home_ready_{false};
        double alt_at_home_{0.0};
        long int alt_home_time_ms_{0};
        bool alt_home_latched_event_{false};

    public:
        
        STORE store;
        
        void system_init(DATA &dat);
        void ekf_step(DATA &dat);       
        bool get_ekf_initialized_state();
        void set_ekf_initialized_statet(bool state);
        double get_yaw_at_home() const { return yaw_at_home; }
        bool altitude_home_ready() const { return alt_home_ready_; }
        double altitude_home_internal() const { return alt_at_home_; }
        long int altitude_home_time_ms() const { return alt_home_time_ms_; }
        /** True once after a new altitude-home latch; clears the event. */
        bool consume_altitude_home_latched_event();
        void reset_altitude_home(); 
        LOCALSLAM_DATA get_lslam_data();
        bool get_KeyFrame(KEYFRAME &KF);
        bool get_KeyFrameCL(KEYFRAME &KF);
        bool get_RobotState(arma::vec::fixed<13> &x_r,double &radius_uncertainty);
        /** Consistent robot-state + 13x13 P snapshot (clears NewRobotState_available). */
        bool get_RobotStateSnapshot(
          arma::vec::fixed<13> & x_r,
          arma::mat::fixed<13, 13> & P_r,
          double & radius_uncertainty);
        /**
         * Read-only copy of robot state and 13×13 covariance (does not clear
         * NewRobotState_available). Used by regression tests / diagnostics.
         */
        void copy_robot_state_and_cov(
          arma::vec::fixed<13> & x_r,
          arma::mat::fixed<13, 13> & P_r) const;
        /**
         * Apply Global SLAM translation delta (internal nav). Returns the
         * actual applied components after GS_/CL_ axis gates. Does not modify P.
         * close_loop resets Pc only when at least one axis is applied.
         */
        struct PosDeltaApplyResult
        {
          bool applied{false};
          double applied_dx{0.0};
          double applied_dy{0.0};
          double applied_dz{0.0};
          bool reset_pc{false};
        };
        PosDeltaApplyResult Update_pos_with_delta(
          arma::vec::fixed<3> delta_pos, string type);
        void SetCompute_Pc_flag(bool state);
        void set_ekf_visual_update(bool state);

        /** A8: apply intrinsics into PAR + cam_parameters (pointer into PAR.distortions). */
        void set_camera_intrinsics(
          int img_cols,
          int img_rows,
          double fc_u,
          double fc_v,
          double cc_u,
          double cc_v,
          double alpha_c,
          const std::vector<double> & distortions);

        /**
         * G3: identity used when selecting keyframes. Call on session create and
         * whenever epoch_id bumps after successful system_init. Resets the
         * per-epoch keyframe_id counter.
         */
        void set_reference_identity(
          const slam_identity::UuidBytes & session_id, uint64_t epoch_id);

        slam_identity::UuidBytes reference_session_id() const { return ref_session_id_; }
        uint64_t reference_epoch_id() const { return ref_epoch_id_; }

        /** Highest assigned keyframe_id this epoch (0 if none). LC-CONSISTENCY-A3. */
        uint64_t last_generated_keyframe_id() const
        {
          return next_keyframe_id_ > 1 ? (next_keyframe_id_ - 1) : 0;
        }

private:
        slam_identity::UuidBytes ref_session_id_{slam_identity::nil_uuid()};
        uint64_t ref_epoch_id_{0};
        uint64_t next_keyframe_id_{1};

        void stamp_keyframe_identity(KEYFRAME & kf, FRAME * frame, uint8_t stream);
};


#endif