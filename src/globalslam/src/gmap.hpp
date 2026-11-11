#ifndef GMAP_H
#define GMAP_H

#include <armadillo>
#include "parameters.hpp"
#include "gmap_relative_kf_init.hpp"
#include "../../common/Vision/vision.hpp"
#include "globalslam_types.hpp"
#include <opencv2/opencv.hpp>

class GMAP
{

    public:        
                
                   
        
        GMAP(parameters &par) // constructor
        {           
            PAR = par;
            InitGmap();
        }
        
         

    private:
        
              

        GLOBAL_MAP Gmap; // Global map structure

        parameters PAR; // parameters structure
        CAM cam_parameters;

        arma::vec::fixed<3> Delta_kf_n;

        std::vector<KEYFRAME> Kf_buffer; // for temporaly storing Keyframes

        //int idx_kf_min_ref_for_search_matches;        
        
        
        //--- for initializing new map points
        std::vector<int64> Init_new_map_points(); 
        arma::vec::fixed<3> Triangulate_sigle_3d_point(cv::Point2f &uv1d,cv::Point2f &uv2d,arma::mat::fixed<3,3> &Rn2c_1,arma::vec::fixed<3> &t_c2n_1,arma::mat::fixed<3,3> &Rn2c_2,arma::vec::fixed<3> &t_c2n_2 );

        //--- for matching map points into Keyframes
        void Match_Gmap_points_into_new_KF();        
        void Match_New_points_into_previous_KF(std::vector<int64> &idx_new_points, std::vector<int> &idx_kf_m, std::vector<int64> &idx_pt_m );
         
        // --- for deleting objects of the global map   
        void Delete_points(std::vector<std::vector<int>> &kf_vg_info);        
        
        // --- for bundle adjusment
        void Bundle_adjustment();
        

        // --- For update and get info from the visibility graph 
        void Update_Visibility_Graph_with_new_KF();
        void Update_Visibility_Graph_with_matches_of_new_points(std::vector<int64> &idx_new_points, std::vector<int> &idx_kf_m, std::vector<int64> &idx_pt_m );         
        void Update_Visibility_Graph_with_deleted_points(std::vector<std::vector<int>> &kf_vg_info);
        std::vector<int> Get_idx_visually_linked_to_KF(int idx_KF);
        std::vector<int> Get_n_oldest_visually_linked_kf(int idx_kf, int n_oldest, int min_v_strength ) ;      
        void  Check_visibiliy_graph(int &idx_direct_link, int &idx_indirect_link );

        double total_init_anchors;
        double total_delete_anchors;

        // INIT-D4 birth gate diagnostics (always counted; gate itself default-off).
        int64_t birth_gate_attempts{0};
        int64_t birth_gate_accepted{0};
        int64_t birth_gate_rejected{0};

        // BA-D6: EKF-at-admission camera pose of the previous KF (for relative init).
        bool have_prev_ekf_cam_pose_{false};
        gmap_relative_kf_init::CamPose prev_ekf_cam_pose_{};
    
    public:
        void InitGmap();

        void  Add_Kf(KEYFRAME &kf);

        void  Update();

        void Get_GlobalMap(GLOBAL_MAP &Gmap_c);
        void Set_GlobalMap(GLOBAL_MAP &Gmap_c);

        arma::vec::fixed<3>  Get_delta_pos();

        void Get_birth_gate_counts(
          int64_t & attempts, int64_t & accepted, int64_t & rejected) const
        {
          attempts = birth_gate_attempts;
          accepted = birth_gate_accepted;
          rejected = birth_gate_rejected;
        }

        STOREG store;

    
};


#endif



















