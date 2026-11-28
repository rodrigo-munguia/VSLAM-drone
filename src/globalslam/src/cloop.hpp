#ifndef GLOOP_H
#define GLOOP_H

#include <armadillo>
#include "parameters.hpp"
#include "../../common/Vision/vision.hpp"
#include "globalslam_types.hpp"
#include "g6b_home_tag_geometry.hpp"
#include "g6b_home_tag_capture.hpp"
#include "cloop_visual_retrieve.hpp"
#include "gmap_loop_proposal.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <optional>
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
        
        

        /**
         * M7.5: ephemeral owned detection extract only (installed for Step/Step2).
         * Empty outside an active detection step — not a persistent map replica.
         */
        GLOBAL_MAP Gmap;
        bool has_detection_extract_{false};

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
        /** LC-CONSISTENCY-A3: Local EKF tip used for δ_L (pre-Pose_SLAM). */
        arma::vec::fixed<3> last_sync_p_l_original_;
        uint64_t last_sync_keyframe_id_{0};

        /** M7.4 proposal submission (detection only; no map mutation). */
        std::optional<gmap_loop::LoopProposal> pending_proposal_;
        uint64_t next_proposal_id_{1};
        uint64_t map_lifetime_{0};
        uint64_t correction_epoch_{0};
        bool await_proposal_outcome_{false};
        /** M7.7.2: stamped onto proposals; must match component generation. */
        uint64_t closing_query_generation_{0};

        double travel_dis;
        int n_cloop_intents;
        int n_kf_home;
        bool activate_CL_flag;
        /** Opt-in M7.7.2 detection cadence counters (always updated, cheap). */
        uint64_t diag_matching_attempts_{0};
        uint64_t diag_pnp_attempts_{0};
        uint64_t diag_verified_proposals_{0};
        uint64_t diag_travel_gate_reject_steps_{0};
        uint64_t diag_cooldown_reject_steps_{0};
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

        /**
         * Resolve closing KF index in the installed detection extract by id.
         * Returns -1 if absent.
         */
        int Find_keyframe_index(uint64_t keyframe_id) const;

        /** A2d/A3g: enumerate+match+rank historical visual candidates (no home/unc).
         *  apply_hist_travel_gate: visual Step only (tag Get_matches passes false).
         *  M7.7.2: when closing KF is already in-map, pass its index as
         *  idx_closing_in_map (>=0) so historical search uses that node (not tip)
         *  and does not append a fictitious endpoint. */
        std::vector<globalslam::cloop_visual_retrieve::RankedCandidate>
        Collect_ranked_visual_candidates(
          KEYFRAME & kf_cl, double * t_search_s, int * n_eligible_out = nullptr,
          int * n_hist_rejected_out = nullptr,
          bool apply_hist_travel_gate = true,
          int idx_closing_in_map = -1);
       
        void Get_matches(KEYFRAME &kf_cl, std::vector<int> &idx_pt_matches, std::vector<cv::Point2d> &image_points ,int &idx_kf_matched);
        
        bool Get_pos_of_current_kf(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl, std::vector<int> &idx_pt_matches, std::vector<cv::Point2d> &image_points, int step);
        bool Get_pos_of_current_kf_from_Home_tag(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl);


        /** M7.4: build proposal from verified geometry (no authoritative mutation). */
        bool Build_loop_proposal(
          arma::vec::fixed<3> & pos,
          KEYFRAME & kf_cl,
          int idx_kf_matched,
          gmap_loop::Kind kind,
          const std::vector<int> * inlier_corr_indices,
          const std::vector<int> * idx_pt_matches,
          const std::vector<cv::Point2d> * image_points);

        // Detection helpers that remain compiled (visibility queries / ORB match).
        // Correction orchestration (Update_gmap / Pose_SLAM / Fuse / BA) removed M7.5.
        void Match_Gmap_points_into_KF(int idx_KF);
        void Update_Visibility_Graph_with_new_KF(int idx_KF);
        void Update_Visibility_Graph_A_B_links(
          std::vector<int> & idx_vl_kf_A, std::vector<int> & idx_vl_kf_B);
        void Match_list_of_points_into_KF(
          std::vector<int64> & idx_points, int idx_KF,
          std::vector<int64> & idx_matched_points);
        void Update_Visibility_Graph_with_matched_points_into_KF(
          std::vector<int64> & idx_points, int idx_KF);
        void Match_a_points_with_b_points(
          std::vector<int64> & idx_a_points, std::vector<int64> & idx_b_points,
          std::vector<int64> & idx_matched_a_pts,
          std::vector<int64> & idx_matched_b_pts);

    public:

        void CLOOP_init();

        /**
         * M7.5: install owned detection extract for the next Step/Step2.
         * Must be a deep-owned map (no aliases into authoritative storage).
         */
        void Install_detection_extract(GLOBAL_MAP && owned);
        void Clear_detection_extract();
        bool Has_detection_extract() const { return has_detection_extract_; }

        void Step(KEYFRAME &kf_cl);
        void Step2(KEYFRAME &kf_cl);

        arma::vec::fixed<3>  Get_delta_pos();

        bool Get_close_loop_state();
        void Set_close_loop_state(bool state);

        /** Legacy: true when CLOOP still ran Pose_SLAM locally (pre-M7.4). */
        bool Last_loop_committed() const { return last_pose_slam_committed_; }
        const std::string & Last_loop_source() const { return last_loop_source_; }
        arma::vec::fixed<3> Last_committed_camera() const
        {
          return last_committed_camera_;
        }
        bool Last_match_ba_refine_ran() const { return last_match_ba_refine_ran_; }

        /** M7.4: take ownership of a verified proposal (if any). */
        bool Take_pending_proposal(gmap_loop::LoopProposal & out);
        /** Apply authoritative GMAP outcome to detection bookkeeping. */
        void Apply_proposal_outcome(const gmap_loop::ProposalOutcome & outcome);
        void Set_map_lifetime_and_epoch(uint64_t lifetime, uint64_t epoch);
        void Set_closing_query_generation(uint64_t gen)
        {
          closing_query_generation_ = gen;
        }
        uint64_t Closing_query_generation() const
        {
          return closing_query_generation_;
        }
        uint64_t Diag_matching_attempts() const { return diag_matching_attempts_; }
        uint64_t Diag_pnp_attempts() const { return diag_pnp_attempts_; }
        uint64_t Diag_verified_proposals() const
        {
          return diag_verified_proposals_;
        }

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


