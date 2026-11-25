// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COMPOSITION__GLOBALSLAM_COMPONENT_HPP_
#define COMPOSITION__GLOBALSLAM_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "parameters.hpp"
#include "interfaces/msg/kf.hpp"
#include "interfaces/msg/gmap.hpp"
#include "interfaces/msg/estimator_reference.hpp"
#include "interfaces/srv/l_spos_update.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/srv/correction_coordinate.hpp"
#include "interfaces/msg/correction_applied.hpp"
#include "interfaces/msg/correction_feedback_accepted.hpp"
#include "interfaces/msg/global_loop_committed.hpp"
#include "g6d_source_policy.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "globalslam_types.hpp"
#include "g2_run_control.hpp"
#include "g3_admission.hpp"
#include "g4_viz_conversions.hpp"
#include "g6_coord_dispatch.hpp"
#include "g6b_feedback_completion.hpp"
#include "../../common/slam_identity.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_set>
#include "gmap.hpp"
#include "cloop.hpp"

using namespace std;

namespace globalslam
{

struct CorrectionProvenance
{
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  uint64_t keyframe_id{0};
  uint64_t correction_id{0};
  g6d::CorrectionSource source{g6d::CorrectionSource::Ba};
};

class Gslam : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit Gslam(const rclcpp::NodeOptions & options);

  ~Gslam();

private:
  rclcpp::Subscription<interfaces::msg::Kf>::SharedPtr sub_Kf_;
  rclcpp::Subscription<interfaces::msg::Kf>::SharedPtr sub_Kf_cl_;
  rclcpp::Subscription<interfaces::msg::EstimatorReference>::SharedPtr sub_estimator_ref_;

  void Kf_callback(const interfaces::msg::Kf & msg);
  void Kf_cl_callback(const interfaces::msg::Kf & msg);
  void EstimatorReference_callback(const interfaces::msg::EstimatorReference & msg);

  bool try_admit_and_convert_kf(
    const interfaces::msg::Kf & msg, KEYFRAME & kf_out, const char * stream_name);

  rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv_globalslam_run_;
  void Handle_globalslam_run_service(
    const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,
    std::shared_ptr<interfaces::srv::SimpleServ::Response> response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  void Handle_enable_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void Handle_reset_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /** Shared implementation for ~/reset and SimpleServ 'i'. */
  void request_reset();
  /** Clear GS state due to EstimatorReference invalid/identity change. */
  void clear_for_reference_change(const char * reason);

  rclcpp::Client<interfaces::srv::LSposUpdate>::SharedPtr client_local_slam_pos_update_;
  /**
   * Dispatch LSposUpdate asynchronously. Response callback is generation-gated.
   * G6a: may require CorrectionCoordinate pause ack before dispatch.
   * Close-loop completion prefers coordinated release(signal_close_loop) over
   * legacy SimpleServ 'c' when coordination was used.
   */
  void Send_local_slam_pos_update(
    arma::vec::fixed<3> & Delta_kf_n,
    g6d::CorrectionSource source,
    const CorrectionProvenance & provenance,
    uint64_t dispatch_generation,
    bool notify_control_on_apply);
  void Publish_global_loop_committed(
    const CorrectionProvenance & provenance,
    uint64_t map_revision,
    const std::string & source,
    const arma::vec::fixed<3> & committed_camera,
    bool match_ba_refine_ran);

  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_control_run_;
  rclcpp::Client<interfaces::srv::CorrectionCoordinate>::SharedPtr client_correction_coord_;
  rclcpp::Subscription<interfaces::msg::CorrectionApplied>::SharedPtr sub_correction_applied_;
  rclcpp::Subscription<interfaces::msg::CorrectionFeedbackAccepted>::SharedPtr
    sub_correction_feedback_accepted_;
  void Send_control_loop_closed_signal();
  bool Prepare_control_pause(
    const CorrectionProvenance & provenance, const std::string & type);
  void Release_or_abort_control(
    const CorrectionProvenance & provenance,
    const std::string & type,
    bool applied,
    bool abort,
    bool resume_if_owned);
  /** G6b.1: finish coordinated txn from FeedbackAccepted / timeout (never block executor). */
  void Complete_coordinated_txn_locked(
    const CorrectionProvenance & provenance,
    const std::string & type,
    uint64_t dispatch_generation,
    g6b_fb::TxnPhase phase);
  void Arm_feedback_timeout_timer(uint64_t correction_id);
  void Cancel_feedback_timeout_timer();
  void CorrectionApplied_callback(const interfaces::msg::CorrectionApplied & msg);
  void CorrectionFeedbackAccepted_callback(
    const interfaces::msg::CorrectionFeedbackAccepted & msg);

  rclcpp::Publisher<interfaces::msg::Gmap>::SharedPtr pub_gmap_data_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_keyframes_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_landmarks_;
  rclcpp::Publisher<interfaces::msg::GlobalLoopCommitted>::SharedPtr
    pub_global_loop_committed_;
  void Publish_gmap_data();
  /** Copy committed Gmap under lock; serialize/publish Path+cloud unlocked. */
  void Publish_standard_viz(uint64_t generation);
  /** Latch empty Path/cloud for late subscribers after reset/ref change. */
  void Publish_empty_standard_viz(uint64_t generation);
  bool Capture_map_snapshot_locked(g4_viz::MapSnapshot & snap, uint64_t generation);

  void setParameters();

  std::thread gmap_loop_;
  std::thread cloop_loop_;
  void GMAP_LOOP();
  void CLOOP_LOOP();
  void stop_workers();

  std::atomic<bool> workers_run_{true};
  /** High 32 bits of correction_id — unique per GS process instance. */
  uint32_t correction_boot_nonce_{0};
  std::atomic<uint32_t> next_correction_seq_{1};

  GLOBAL_MAP Gmap;

  std::vector<KEYFRAME> Kf_buffer;
  KEYFRAME kf_cl_last;
  bool new_kf_cl{false};
  bool gmap_updated{false};

  parameters PAR;
  bool accept_missing_source_stamp_{false};
  std::string estimator_reference_topic_{"/EKFslam/estimator_reference"};
  std::string public_frame_id_{"odom"};
  int path_max_poses_{0};
  int landmark_cloud_max_points_{0};
  /** G6a: required | absent_ok | offline_bypass (see g6_coord_dispatch.hpp). */
  std::string coordinate_mode_str_{"required"};
  int coordinate_pause_timeout_ms_{2000};
  int coordinate_feedback_barrier_timeout_ms_{2000};
  std::string correction_applied_topic_{"/EKFslam/correction_applied"};
  std::string correction_feedback_accepted_topic_{"correction_feedback_accepted"};

  G2RunControl run_ctrl_;
  /** Guards pending coordinated txn + early FeedbackAccepted latch (G6b.1). */
  std::mutex mutex_corr_barrier_;
  g6b_fb::PendingTxn pending_corr_txn_;
  std::deque<uint64_t> early_feedback_ids_;
  CorrectionProvenance pending_corr_provenance_{};
  rclcpp::TimerBase::SharedPtr feedback_timeout_timer_;
  ActiveReference active_ref_;
  std::unordered_set<uint64_t> seen_keyframe_ids_;

  // Lock order (never invert): run_ctrl_ → rx_kf → rx_kf_cl → get_gm → logs/flags.
  // mutex_run_ctrl_ also guards active_ref_ / seen_keyframe_ids_.
  std::mutex mutex_run_ctrl_;
  std::mutex mutex_rx_kf;
  std::mutex mutex_rx_kf_cl;
  std::mutex mutex_get_gm;
  std::mutex mutex_log_gmap;
  std::mutex mutex_log_cloop;
  std::mutex mutex_rechome;
  std::mutex mutex_activate_CL;
  std::mutex mutex_home_capture_q_;

  bool gmap_get_log_flag{false};
  bool cloop_get_log_flag{false};
  bool loop_closed_flag{false};

  /**
   * LC-CONSISTENCY-A3: GMAP is stack-local to GMAP_LOOP; sync commands are
   * queued here and drained on the GMAP worker (mutex_visual_sync_).
   */
  enum class VisualSyncCmdKind : uint8_t
  {
    None = 0,
    Suspend = 1,
    Open = 2,
    Applied = 3,
  };
  struct VisualSyncCmd
  {
    VisualSyncCmdKind kind{VisualSyncCmdKind::None};
    double dx{0.0};
    double dy{0.0};
    uint64_t watermark{0};
    uint64_t correction_id{0};
  };
  std::mutex mutex_visual_sync_;
  VisualSyncCmd visual_sync_cmd_{};
  void Queue_visual_sync_cmd(const VisualSyncCmd & cmd);
  void Drain_visual_sync_cmd(GMAP & gmap);
  /** Legacy end-of-RecHome: finalize capture + stamp n_kf_home ('u'). */
  bool rechome_flag{false};
  /** Start RecHome capture window ('b'). */
  bool rechome_begin_flag_{false};
  bool activate_CL_flag{false};
  /** Original LS KF snapshots for home-tag capture (map + CL streams). */
  std::vector<KEYFRAME> home_capture_kf_q_;
  bool home_capture_active_{false};
  bool home_capture_finalize_pending_{false};
  bool home_capture_finalize_result_{false};
  bool home_capture_finalize_done_{false};
  int home_capture_finalize_idle_iters_{0};
  int home_capture_finalize_wait_iters_{0};
  bool home_capture_invalidate_{false};

  void clear_pending_keyframes_locked();
  void log_data_g(STOREG & data);
  void log_data_c(STOREC & data);
  void mean_std(std::vector<double> & data, double & mean, double & std, double & sum);
  void log_data_to_file(
    string file_name, std::pair<std::vector<double>, std::vector<double>> & data);
};

}  // namespace globalslam

#endif  // COMPOSITION__GLOBALSLAM_COMPONENT_HPP_
