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

#include "globalslam_component.hpp"
#include "rclcpp/rclcpp.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include "../../common/Transforms/quat2R.hpp"
#include "gmap.hpp"
#include "../../common/Transforms/Ra2b_TO_Quat_a2b.hpp"
#include "aux_functions.hpp"
#include "../../common/ros_slam_identity.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <future>

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

namespace globalslam
{

Gslam::Gslam(const rclcpp::NodeOptions & options)
: Node("gslam", options)
{
  setParameters();

  sub_Kf_ = this->create_subscription<interfaces::msg::Kf>(
    "Kf_topic", 10, std::bind(&Gslam::Kf_callback, this, _1));
  sub_Kf_cl_ = this->create_subscription<interfaces::msg::Kf>(
    "Kf_cl_topic", 10, std::bind(&Gslam::Kf_cl_callback, this, _1));

  {
    rclcpp::QoS qos_ref(rclcpp::KeepLast(1));
    qos_ref.reliable();
    qos_ref.transient_local();
    sub_estimator_ref_ =
      this->create_subscription<interfaces::msg::EstimatorReference>(
        estimator_reference_topic_, qos_ref,
        std::bind(&Gslam::EstimatorReference_callback, this, _1));
  }

  srv_globalslam_run_ = this->create_service<interfaces::srv::SimpleServ>(
    "globalslam_run_service",
    std::bind(&Gslam::Handle_globalslam_run_service, this, _1, _2));
  srv_enable_ = this->create_service<std_srvs::srv::SetBool>(
    "~/enable", std::bind(&Gslam::Handle_enable_service, this, _1, _2));
  srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
    "~/reset", std::bind(&Gslam::Handle_reset_service, this, _1, _2));

  client_local_slam_pos_update_ =
    this->create_client<interfaces::srv::LSposUpdate>("local_slam_pos_update");
  client_control_run_ =
    this->create_client<interfaces::srv::SimpleServ>("control_run_service");
  client_correction_coord_ =
    this->create_client<interfaces::srv::CorrectionCoordinate>(
      "control_correction_coordinate");
  {
    rclcpp::QoS qos_corr(rclcpp::KeepLast(1));
    qos_corr.reliable();
    qos_corr.transient_local();
    sub_correction_applied_ =
      this->create_subscription<interfaces::msg::CorrectionApplied>(
        correction_applied_topic_, qos_corr,
        std::bind(&Gslam::CorrectionApplied_callback, this, _1));
    sub_correction_feedback_accepted_ =
      this->create_subscription<interfaces::msg::CorrectionFeedbackAccepted>(
        correction_feedback_accepted_topic_, qos_corr,
        std::bind(&Gslam::CorrectionFeedbackAccepted_callback, this, _1));
  }

  pub_gmap_data_ = create_publisher<interfaces::msg::Gmap>("gmap_data_topic", 10);

  // Transient-local so late RViz2/Plot2 subscribers receive the last snapshot.
  {
    rclcpp::QoS qos_viz(rclcpp::KeepLast(1));
    qos_viz.reliable();
    qos_viz.transient_local();
    pub_global_keyframes_ =
      create_publisher<nav_msgs::msg::Path>("~/global_keyframes", qos_viz);
    pub_global_landmarks_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/global_landmarks", qos_viz);
    pub_global_loop_committed_ =
      create_publisher<interfaces::msg::GlobalLoopCommitted>(
        "~/global_loop_committed", qos_viz);
  }

  // Flags before workers start (workers read these).
  gmap_updated = false;
  new_kf_cl = false;
  loop_closed_flag = false;
  gmap_get_log_flag = false;
  cloop_get_log_flag = false;
  rechome_flag = false;
  rechome_begin_flag_ = false;
  home_capture_active_ = false;
  home_capture_finalize_pending_ = false;
  home_capture_finalize_done_ = false;
  home_capture_finalize_result_ = false;
  home_capture_finalize_idle_iters_ = 0;
  home_capture_finalize_wait_iters_ = 0;
  home_capture_invalidate_ = false;
  activate_CL_flag = false;
  workers_run_ = true;
  {
    const auto t = static_cast<uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    correction_boot_nonce_ = static_cast<uint32_t>(
      (t ^ static_cast<uint64_t>(reinterpret_cast<uintptr_t>(this))) & 0xffffffffu);
    if (correction_boot_nonce_ == 0) {
      correction_boot_nonce_ = 1;
    }
  }

  gmap_loop_ = std::thread(&Gslam::GMAP_LOOP, this);
  cloop_loop_ = std::thread(&Gslam::CLOOP_LOOP, this);

  RCLCPP_INFO(
    this->get_logger(),
    "Gslam ready: G6d matrix [%s] estimator_reference=%s "
    "accept_missing_source_stamp=%d public_frame=%s "
    "birth_reproj_gate=%d max_rms_px=%.3f relative_kf_pose=%d "
    "Path/cloud body-origin keyframes + sparse landmarks "
    "(GS restart does not replay past KFs — prior global map/home not reconstructed). "
    "Enable processing: ros2 service call /gslam/enable std_srvs/srv/SetBool \"{data: true}\"",
    run_ctrl_.sources.matrix_summary().c_str(),
    estimator_reference_topic_.c_str(),
    accept_missing_source_stamp_ ? 1 : 0,
    public_frame_id_.c_str(),
    PAR.Init_birth_reproj_gate_enable ? 1 : 0,
    PAR.Init_birth_reproj_max_rms_px,
    PAR.Init_relative_kf_pose_enable ? 1 : 0);
}

Gslam::~Gslam()
{
  stop_workers();
}

void Gslam::stop_workers()
{
  workers_run_ = false;
  if (gmap_loop_.joinable()) {
    gmap_loop_.join();
  }
  if (cloop_loop_.joinable()) {
    cloop_loop_.join();
  }
}

void Gslam::clear_pending_keyframes_locked()
{
  Kf_buffer.clear();
  new_kf_cl = false;
  gmap_updated = false;
  loop_closed_flag = false;
}

void Gslam::clear_for_reference_change(const char * reason)
{
  {
    std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
    g6b_fb::clear_txn(pending_corr_txn_);
    early_feedback_ids_.clear();
    Cancel_feedback_timeout_timer();
  }

  uint64_t gen = 0;
  {
    std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
    std::lock_guard<std::mutex> lock_kf(mutex_rx_kf);
    std::lock_guard<std::mutex> lock_cl(mutex_rx_kf_cl);
    std::lock_guard<std::mutex> lock_gm(mutex_get_gm);
    run_ctrl_.request_reset();
    gen = run_ctrl_.reset_generation;
    seen_keyframe_ids_.clear();
    clear_pending_keyframes_locked();
    Gmap.AnchorsDATA.clear();
    Gmap.KeyFDATA.clear();
    Gmap.Vgraph.clear();
  }
  {
    std::lock_guard<std::mutex> lock_q(mutex_home_capture_q_);
    home_capture_kf_q_.clear();
  }
  {
    std::lock_guard<std::mutex> lock_rh(mutex_rechome);
    home_capture_active_ = false;
    home_capture_finalize_pending_ = false;
    home_capture_finalize_done_ = false;
    home_capture_finalize_result_ = false;
    home_capture_invalidate_ = false;
    rechome_begin_flag_ = false;
    rechome_flag = false;
  }
  // Clear latched Path/cloud so late subscribers cannot see prior-epoch geometry.
  Publish_empty_standard_viz(gen);
  RCLCPP_INFO(
    this->get_logger(),
    "Gslam cleared for reference change (%s); generation=%llu "
    "(in-flight steps cannot commit across this generation; standard viz emptied)",
    reason, static_cast<unsigned long long>(gen));
}

void Gslam::request_reset()
{
  clear_for_reference_change("~/reset or SimpleServ 'i'");
}

void Gslam::GMAP_LOOP()
{
  GMAP gmap(PAR);
  cout << "-> Global map thread running... " << endl;
  int c_kf = 0;

  while (workers_run_) {
    uint64_t step_gen = 0;
    bool processing = false;
    {
      std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
      if (run_ctrl_.consume_reinit_gmap()) {
        gmap.InitGmap();
        c_kf = 0;
      }
      step_gen = run_ctrl_.reset_generation;
      processing = run_ctrl_.sources.processing_enabled;
    }

    {
      // Lock order: run_ctrl → kf → gm (avoid gm→kf deadlock with reset).
      std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
      std::lock_guard<std::mutex> lock_kf(mutex_rx_kf);
      std::lock_guard<std::mutex> lock_gm(mutex_get_gm);
      if (loop_closed_flag) {
        loop_closed_flag = false;
        if (run_ctrl_.results_belong_to_generation(step_gen)) {
          gmap.Set_GlobalMap(Gmap);
        }
        Kf_buffer.clear();
      }
    }

    bool new_kf = false;
    CorrectionProvenance step_prov;
    if (processing) {
      std::lock_guard<std::mutex> lock_kf(mutex_rx_kf);
      if (!Kf_buffer.empty()) {
        for (size_t i = 0; i < Kf_buffer.size(); i++) {
          gmap.Add_Kf(Kf_buffer[i]);
          // Provenance from the last KF in this batch (originating computation).
          step_prov.session_id = Kf_buffer[i].session_id;
          step_prov.epoch_id = Kf_buffer[i].epoch_id;
          step_prov.keyframe_id = Kf_buffer[i].keyframe_id;
        }
        Kf_buffer.clear();
        new_kf = true;
      }
    }

    if (new_kf) {
      gmap.Update();

      // Validate generation+enable and commit shared state under one critical
      // section. Capture any outbound correction decision here; send outside
      // locks (async; already-dispatched requests cannot be recalled).
      // Stamp provenance from the KF that originated this step — never the
      // live EstimatorReference at send time.
      bool send_ba = false;
      bool publish_std = false;
      arma::vec::fixed<3> delta_pos;
      CorrectionProvenance send_prov = step_prov;
      send_prov.source = g6d::CorrectionSource::Ba;
      {
        std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
        std::lock_guard<std::mutex> lock_gm(mutex_get_gm);
        const bool commit = run_ctrl_.results_belong_to_generation(step_gen) &&
          run_ctrl_.sources.processing_enabled &&
          active_ref_.valid &&
          slam_identity::equal(step_prov.session_id, active_ref_.session_id) &&
          step_prov.epoch_id == active_ref_.epoch_id;
        if (commit) {
          const bool skip_for_cl = loop_closed_flag;
          if (!skip_for_cl) {
            gmap.Get_GlobalMap(Gmap);
            if (c_kf % 4 == 0) {
              Publish_gmap_data();
            }
            gmap_updated = true;
            publish_std = true;
            if (run_ctrl_.should_emit_for(g6d::CorrectionSource::Ba) &&
              !home_capture_active_)
            {
              delta_pos = gmap.Get_delta_pos();
              const uint32_t seq = next_correction_seq_++;
              send_prov.correction_id =
                (static_cast<uint64_t>(correction_boot_nonce_) << 32) | seq;
              send_ba = true;
            } else if (home_capture_active_ &&
              run_ctrl_.should_emit_for(g6d::CorrectionSource::Ba))
            {
              RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Withholding BA correction while home-tag capture is active");
            }
          }
          c_kf++;
        }
      }
      // Serialize/publish standard Path+cloud outside worker locks.
      if (publish_std) {
        Publish_standard_viz(step_gen);
      }
      if (send_ba) {
        Send_local_slam_pos_update(
          delta_pos, g6d::CorrectionSource::Ba, send_prov, step_gen, false);
      }
    }

    {
      std::lock_guard<std::mutex> lock_log(mutex_log_gmap);
      if (gmap_get_log_flag) {
        gmap_get_log_flag = false;
        log_data_g(gmap.store);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Gslam::CLOOP_LOOP()
{
  CLOOP cloop(PAR);
  cout << "-> Close-loop thread running... " << endl;

  while (workers_run_) {
    uint64_t step_gen = 0;
    bool processing = false;
    {
      std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
      if (run_ctrl_.consume_reinit_cloop()) {
        cloop.CLOOP_init();
      }
      step_gen = run_ctrl_.reset_generation;
      processing = run_ctrl_.sources.processing_enabled;
    }

    {
      std::lock_guard<std::mutex> lock_gm(mutex_get_gm);
      if (gmap_updated) {
        gmap_updated = false;
        cloop.Set_GlobalMap(Gmap);
      }
    }

    bool have_cl_kf = false;
    KEYFRAME kf_cl;
    if (processing) {
      std::lock_guard<std::mutex> lock_cl(mutex_rx_kf_cl);
      if (new_kf_cl) {
        new_kf_cl = false;
        kf_cl = kf_cl_last;
        have_cl_kf = true;
      }
    }

    // Safety net: correction applied while capture active → fail the window.
    {
      std::lock_guard<std::mutex> lock_rh(mutex_rechome);
      if (home_capture_invalidate_) {
        home_capture_invalidate_ = false;
        cloop.Invalidate_home_tag_capture("correction_during_capture");
        home_capture_active_ = false;
        home_capture_finalize_pending_ = false;
        home_capture_finalize_done_ = true;
        home_capture_finalize_result_ = false;
      }
    }

    // RecHome begin: start capture from original LS KF snapshots.
    {
      std::lock_guard<std::mutex> lock_rh(mutex_rechome);
      if (rechome_begin_flag_) {
        slam_identity::UuidBytes sid;
        uint64_t epoch = 0;
        bool ok = false;
        {
          std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
          ok = active_ref_.valid;
          sid = active_ref_.session_id;
          epoch = active_ref_.epoch_id;
        }
        std::string reason;
        if (ok && cloop.Begin_home_tag_capture(sid, epoch, reason)) {
          home_capture_active_ = true;
          home_capture_finalize_done_ = false;
          home_capture_finalize_result_ = false;
        } else if (ok) {
          RCLCPP_WARN(
            this->get_logger(),
            "RecHome begin refused: %s", reason.c_str());
          // Already ready: keep frozen reference; do not restart silently.
          home_capture_active_ = false;
          {
            std::lock_guard<std::mutex> lock_q(mutex_home_capture_q_);
            home_capture_kf_q_.clear();
          }
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "RecHome begin ignored: no valid EstimatorReference");
        }
        rechome_begin_flag_ = false;
      }
    }

    // RecHome end: request finalize (cutoff = max KF id/time already seen).
    {
      std::lock_guard<std::mutex> lock_rh(mutex_rechome);
      if (rechome_flag) {
        slam_identity::UuidBytes sid;
        uint64_t epoch = 0;
        bool ok = false;
        {
          std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
          ok = active_ref_.valid;
          sid = active_ref_.session_id;
          epoch = active_ref_.epoch_id;
        }
        if (ok) {
          cloop.Set_n_kf_rechome(sid, epoch);
          if (cloop.Home_tag_capture_is_recording()) {
            cloop.Request_home_tag_capture_finalize();
            home_capture_finalize_pending_ = true;
            home_capture_finalize_idle_iters_ = 0;
            home_capture_finalize_wait_iters_ = 0;
          } else if (cloop.Home_tag_spatial_ready()) {
            home_capture_finalize_result_ = true;
            home_capture_finalize_done_ = true;
            home_capture_active_ = false;
          } else {
            home_capture_finalize_result_ = false;
            home_capture_finalize_done_ = true;
            home_capture_active_ = false;
            RCLCPP_WARN(
              this->get_logger(),
              "RecHome finalize: not recording and no ready reference");
          }
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "RecHome finalize ignored: no valid EstimatorReference");
        }
        rechome_flag = false;
      }
    }

    // Drain capture queue AFTER finalize request so in-flight KFs ≤ cutoff apply.
    // Observations use admitted KEYFRAME poses (LS selection), never Gmap.KeyFDATA.
    {
      std::vector<KEYFRAME> batch;
      {
        std::lock_guard<std::mutex> lock_q(mutex_home_capture_q_);
        batch.swap(home_capture_kf_q_);
      }
      for (const auto & kf : batch) {
        cloop.Offer_home_tag_capture_kf(kf);
      }
      if (home_capture_finalize_pending_ &&
        cloop.Home_tag_capture_is_recording() &&
        !home_capture_finalize_done_)
      {
        ++home_capture_finalize_wait_iters_;
        // Finalize when the capture queue stayed empty for two consecutive
        // drains (queued pre-cutoff KFs consumed), or after ~1 s bound.
        if (batch.empty()) {
          ++home_capture_finalize_idle_iters_;
        } else {
          home_capture_finalize_idle_iters_ = 0;
        }
        if (home_capture_finalize_idle_iters_ >= 2 ||
          home_capture_finalize_wait_iters_ >= 20)
        {
          const bool ready_now = cloop.Try_finalize_home_tag_capture();
          home_capture_finalize_result_ = ready_now;
          home_capture_finalize_done_ = true;
          home_capture_finalize_pending_ = false;
          home_capture_active_ = false;
        }
      }
    }

    if (have_cl_kf) {
      {
        std::lock_guard<std::mutex> lock_ac(mutex_activate_CL);
        if (activate_CL_flag) {
          activate_CL_flag = false;
          cloop.Set_Activate_Deactivate_CL(true);
        }
      }

      bool run_tag = false;
      bool run_vis = false;
      {
        std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
        run_tag = run_ctrl_.sources.should_run_tag_loop();
        run_vis = run_ctrl_.sources.should_run_visual_loop();
      }
      if (run_tag) {
        cloop.Step2(kf_cl);
      } else if (run_vis) {
        cloop.Step(kf_cl);
      }

      if (cloop.Get_close_loop_state() && cloop.Last_loop_committed()) {
        cloop.Set_close_loop_state(false);

        const std::string loop_src = cloop.Last_loop_source();
        const g6d::CorrectionSource corr_src =
          (loop_src == "visual_loop") ? g6d::CorrectionSource::VisualLoop
                                      : g6d::CorrectionSource::TagLoop;
        const arma::vec::fixed<3> committed_cam = cloop.Last_committed_camera();
        const bool refine_ran = cloop.Last_match_ba_refine_ran();

        bool send_correction = false;
        bool notify_control_on_apply = false;
        bool publish_std = false;
        bool publish_committed_event = false;
        uint64_t map_revision = 0;
        arma::vec::fixed<3> delta_pos;
        CorrectionProvenance send_prov;
        send_prov.session_id = kf_cl.session_id;
        send_prov.epoch_id = kf_cl.epoch_id;
        send_prov.keyframe_id = kf_cl.keyframe_id;
        send_prov.source = corr_src;
        {
          // Commit + dispatch decision under one critical section.
          // Control 'c' is deferred until LS reports applied=true (G5).
          // Live pause-before-dispatch remains a separate enablement blocker.
          std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
          std::lock_guard<std::mutex> lock_gm(mutex_get_gm);
          const bool commit = run_ctrl_.results_belong_to_generation(step_gen) &&
            run_ctrl_.sources.processing_enabled &&
            active_ref_.valid &&
            slam_identity::equal(kf_cl.session_id, active_ref_.session_id) &&
            kf_cl.epoch_id == active_ref_.epoch_id;
          if (commit) {
            cloop.Get_GlobalMap(Gmap);
            Publish_gmap_data();
            loop_closed_flag = true;
            publish_std = true;
            ++run_ctrl_.sources.map_revision;
            map_revision = run_ctrl_.sources.map_revision;
            publish_committed_event = true;
            if (run_ctrl_.should_emit_for(corr_src) && !home_capture_active_) {
              delta_pos = cloop.Get_delta_pos();
              const uint32_t seq = next_correction_seq_++;
              send_prov.correction_id =
                (static_cast<uint64_t>(correction_boot_nonce_) << 32) | seq;
              send_correction = true;
              notify_control_on_apply = run_ctrl_.should_emit_control_for(corr_src);
            } else if (home_capture_active_ && run_ctrl_.should_emit_for(corr_src)) {
              RCLCPP_INFO(
                this->get_logger(),
                "Global loop committed (%s); withholding Local feedback "
                "(home-tag capture active)",
                loop_src.c_str());
            } else {
              RCLCPP_INFO(
                this->get_logger(),
                "Global loop committed (%s); Local SLAM feedback disabled",
                loop_src.c_str());
            }
          }
        }
        if (publish_committed_event) {
          Publish_global_loop_committed(
            send_prov, map_revision, loop_src, committed_cam, refine_ran);
        }
        if (publish_std) {
          Publish_standard_viz(step_gen);
        }
        if (send_correction) {
          Send_local_slam_pos_update(
            delta_pos, corr_src, send_prov, step_gen, notify_control_on_apply);
        }
      } else if (cloop.Get_close_loop_state()) {
        cloop.Set_close_loop_state(false);
        RCLCPP_WARN(
          this->get_logger(),
          "No global loop committed (Pose_SLAM produced no deltas)");
      }
    }

    {
      std::lock_guard<std::mutex> lock_log(mutex_log_cloop);
      if (cloop_get_log_flag) {
        cloop_get_log_flag = false;
        log_data_c(cloop.store);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void Gslam::CorrectionApplied_callback(const interfaces::msg::CorrectionApplied & msg)
{
  // Primary policy: withhold corrections while recording. Safety net if one
  // still applies: invalidate the active capture window (not a Ready freeze).
  if (msg.applied && home_capture_active_) {
    std::lock_guard<std::mutex> lock_rh(mutex_rechome);
    home_capture_invalidate_ = true;
  }
  // Evidence only — completion requires CorrectionFeedbackAccepted (G6b.1).
  RCLCPP_INFO(
    this->get_logger(),
    "[G6b-TXN] saw CorrectionApplied correction_id=%llu snapshot_seq=%llu "
    "applied=%d (awaiting Control feedback acceptance)",
    static_cast<unsigned long long>(msg.correction_id),
    static_cast<unsigned long long>(msg.snapshot_seq),
    msg.applied ? 1 : 0);
}

void Gslam::Cancel_feedback_timeout_timer()
{
  if (feedback_timeout_timer_) {
    feedback_timeout_timer_->cancel();
    feedback_timeout_timer_.reset();
  }
}

void Gslam::Arm_feedback_timeout_timer(uint64_t correction_id)
{
  Cancel_feedback_timeout_timer();
  const int ms = coordinate_feedback_barrier_timeout_ms_;
  feedback_timeout_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(ms),
    [this, correction_id]() {
      CorrectionProvenance prov;
      std::string type;
      uint64_t gen = 0;
      g6b_fb::TxnPhase phase = g6b_fb::TxnPhase::Idle;
      {
        std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
        Cancel_feedback_timeout_timer();
        if (pending_corr_txn_.correction_id != correction_id ||
          pending_corr_txn_.phase != g6b_fb::TxnPhase::AwaitingFeedback)
        {
          return;
        }
        g6b_fb::on_feedback_timeout(pending_corr_txn_);
        phase = pending_corr_txn_.phase;
        prov = pending_corr_provenance_;
        type = pending_corr_txn_.type;
        gen = pending_corr_txn_.dispatch_generation;
        RCLCPP_WARN(
          this->get_logger(),
          "Correction feedback barrier timeout for correction_id=%llu; "
          "holding pause (applied unresolved — no plan resume)",
          static_cast<unsigned long long>(correction_id));
      }
      Complete_coordinated_txn_locked(prov, type, gen, phase);
    });
}

void Gslam::Complete_coordinated_txn_locked(
  const CorrectionProvenance & provenance,
  const std::string & type,
  uint64_t dispatch_generation,
  g6b_fb::TxnPhase phase)
{
  // Caller may hold mutex_corr_barrier_ briefly before calling; we take
  // run_ctrl_ separately. Do not block the executor waiting for topics.
  {
    std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
    if (!run_ctrl_.results_belong_to_generation(dispatch_generation)) {
      Release_or_abort_control(provenance, type, false, true, false);
      std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
      g6b_fb::clear_txn(pending_corr_txn_);
      return;
    }
  }
  if (phase == g6b_fb::TxnPhase::ReadyToRelease) {
    Release_or_abort_control(provenance, type, true, false, true);
    std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
    g6b_fb::clear_txn(pending_corr_txn_);
    Cancel_feedback_timeout_timer();
    RCLCPP_INFO(
      this->get_logger(),
      "[G6b-TXN] coordinated release after feedback correction_id=%llu",
      static_cast<unsigned long long>(provenance.correction_id));
    return;
  }
  if (phase == g6b_fb::TxnPhase::NotApplied) {
    // Definite non-application → abort may restore plan.
    Release_or_abort_control(provenance, type, false, true, true);
    std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
    g6b_fb::clear_txn(pending_corr_txn_);
    Cancel_feedback_timeout_timer();
    return;
  }
  if (phase == g6b_fb::TxnPhase::UnresolvedHold) {
    // Applied (or unknown) without feedback — abort without resume.
    Release_or_abort_control(provenance, type, true, true, false);
    std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
    g6b_fb::clear_txn(pending_corr_txn_);
    Cancel_feedback_timeout_timer();
    return;
  }
}

void Gslam::CorrectionFeedbackAccepted_callback(
  const interfaces::msg::CorrectionFeedbackAccepted & msg)
{
  CorrectionProvenance prov;
  std::string type;
  uint64_t gen = 0;
  bool do_complete = false;
  g6b_fb::TxnPhase phase = g6b_fb::TxnPhase::Idle;
  {
    std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
    const bool ready = g6b_fb::on_feedback_accepted(
      pending_corr_txn_, early_feedback_ids_, msg.correction_id,
      msg.snapshot_seq, msg.feedback_mode);
    RCLCPP_INFO(
      this->get_logger(),
      "[G6b-TXN] CorrectionFeedbackAccepted correction_id=%llu "
      "snapshot_seq=%llu mode=%s phase=%s",
      static_cast<unsigned long long>(msg.correction_id),
      static_cast<unsigned long long>(msg.snapshot_seq),
      msg.feedback_mode.c_str(),
      g6b_fb::phase_name(pending_corr_txn_.phase));
    if (ready) {
      do_complete = true;
      phase = pending_corr_txn_.phase;
      prov = pending_corr_provenance_;
      type = pending_corr_txn_.type;
      gen = pending_corr_txn_.dispatch_generation;
      Cancel_feedback_timeout_timer();
    }
  }
  if (do_complete) {
    Complete_coordinated_txn_locked(prov, type, gen, phase);
  }
}

bool Gslam::Prepare_control_pause(
  const CorrectionProvenance & provenance, const std::string & type)
{
  if (!client_correction_coord_) {
    return false;
  }
  if (!client_correction_coord_->service_is_ready()) {
    return false;
  }
  auto req = std::make_shared<interfaces::srv::CorrectionCoordinate::Request>();
  req->operation = "prepare_pause";
  req->session_id = slam_identity::to_msg(provenance.session_id);
  req->epoch_id = provenance.epoch_id;
  req->transaction_id = provenance.correction_id;
  req->owner_generation = correction_boot_nonce_;
  req->correction_type = type;
  req->signal_close_loop = false;
  req->resume_plan_if_owned = false;
  auto future = client_correction_coord_->async_send_request(req);
  const auto status = future.wait_for(
    std::chrono::milliseconds(coordinate_pause_timeout_ms_));
  if (status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "CorrectionCoordinate prepare_pause timeout");
    return false;
  }
  auto resp = future.get();
  if (!resp || !resp->success || !resp->cmd_source_quiescent) {
    RCLCPP_WARN(
      this->get_logger(),
      "CorrectionCoordinate prepare_pause failed: success=%d quiescent=%d reason=%s",
      resp ? (resp->success ? 1 : 0) : 0,
      resp ? (resp->cmd_source_quiescent ? 1 : 0) : 0,
      resp ? resp->reason.c_str() : "null");
    return false;
  }
  return true;
}

void Gslam::Release_or_abort_control(
  const CorrectionProvenance & provenance,
  const std::string & type,
  bool applied,
  bool abort,
  bool resume_if_owned)
{
  if (!client_correction_coord_ || !client_correction_coord_->service_is_ready()) {
    return;
  }
  auto req = std::make_shared<interfaces::srv::CorrectionCoordinate::Request>();
  req->operation = abort ? "abort" : "release";
  req->session_id = slam_identity::to_msg(provenance.session_id);
  req->epoch_id = provenance.epoch_id;
  req->transaction_id = provenance.correction_id;
  req->owner_generation = correction_boot_nonce_;
  req->correction_type = type;
  req->signal_close_loop = !abort && applied && (type == "close_loop");
  // release: resume when applied+feedback done. abort: only definite non-apply.
  req->resume_plan_if_owned = resume_if_owned;
  (void)client_correction_coord_->async_send_request(req);
}

void Gslam::Send_local_slam_pos_update(
  arma::vec::fixed<3> & Delta_kf_n,
  g6d::CorrectionSource source,
  const CorrectionProvenance & provenance,
  uint64_t dispatch_generation,
  bool notify_control_on_apply)
{
  if (!client_local_slam_pos_update_) {
    return;
  }
  const std::string type = g6d::ls_update_type_for(source);
  const std::string source_name = g6d::source_name(source);
  const bool apply_this = run_ctrl_.should_emit_for(source);
  // Coordination decision outside worker locks (may wait on control).
  const auto mode = g6_coord::parse_coord_mode(coordinate_mode_str_);
  const bool control_ready =
    client_correction_coord_ && client_correction_coord_->service_is_ready();
  bool pause_acked = false;
  bool used_coordination = false;
  {
    auto decision = g6_coord::decide_dispatch(
      apply_this, mode, control_ready, /*pause_acked=*/false);
    if (!decision.may_dispatch && !decision.need_pause) {
      RCLCPP_WARN(
        this->get_logger(),
        "Dropping %s/%s correction: %s", type.c_str(), source_name.c_str(),
        decision.reason.c_str());
      return;
    }
    if (decision.need_pause) {
      used_coordination = true;
      pause_acked = Prepare_control_pause(provenance, type);
      // Re-check generation after possible wait.
      {
        std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
        if (!run_ctrl_.results_belong_to_generation(dispatch_generation) ||
          !run_ctrl_.should_emit_for(source))
        {
          RCLCPP_WARN(
            this->get_logger(),
            "Aborting coordinated correction after pause: generation/gate changed");
          if (pause_acked) {
            // Generation change: unknown apply state — do not resume.
            Release_or_abort_control(provenance, type, false, true, false);
          }
          return;
        }
      }
      decision = g6_coord::decide_dispatch(
        apply_this, mode, control_ready, pause_acked);
      if (!decision.may_dispatch) {
        RCLCPP_WARN(
          this->get_logger(),
          "Dropping %s/%s correction_id=%llu: %s",
          type.c_str(), source_name.c_str(),
          static_cast<unsigned long long>(provenance.correction_id),
          decision.reason.c_str());
        if (pause_acked) {
          // Never dispatched → definite non-application; resume OK.
          Release_or_abort_control(provenance, type, false, true, true);
        }
        return;
      }
    }
  }

  // Non-blocking readiness only — never wait under locks. Once
  // async_send_request returns, the request cannot be recalled.
  if (!client_local_slam_pos_update_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "local_slam_pos_update unavailable; dropping %s/%s correction",
      type.c_str(), source_name.c_str());
    if (used_coordination && pause_acked) {
      Release_or_abort_control(provenance, type, false, true, true);
    }
    return;
  }
  auto request = std::make_shared<interfaces::srv::LSposUpdate::Request>();
  request->delta_x = Delta_kf_n[0];
  request->delta_y = Delta_kf_n[1];
  request->delta_z = Delta_kf_n[2];
  request->type = type;
  request->source = source_name;
  request->session_id = slam_identity::to_msg(provenance.session_id);
  request->epoch_id = provenance.epoch_id;
  request->keyframe_id = provenance.keyframe_id;
  request->correction_id = provenance.correction_id;

  // G6b.1: register before dispatch so early FeedbackAccepted is latched.
  if (used_coordination) {
    std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
    g6b_fb::register_dispatch(
      pending_corr_txn_, early_feedback_ids_, provenance.correction_id,
      dispatch_generation, provenance.epoch_id, type, used_coordination,
      notify_control_on_apply);
    pending_corr_provenance_ = provenance;
  }

  using ServiceT = interfaces::srv::LSposUpdate;
  auto callback =
    [this, dispatch_generation, notify_control_on_apply, type, source,
      used_coordination, provenance](rclcpp::Client<ServiceT>::SharedFuture future)
    {
      // Never block here waiting for FeedbackAccepted — same executor must
      // deliver that subscription (G6b.1 defect).
      bool send_legacy_c = false;
      bool applied = false;
      bool unresolved = false;
      bool stale_generation = false;
      const uint64_t cid = provenance.correction_id;
      {
        std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
        if (!run_ctrl_.results_belong_to_generation(dispatch_generation)) {
          RCLCPP_WARN(
            this->get_logger(),
            "Ignoring stale LSposUpdate response after reset "
            "(correction_id=%llu generation=%llu)",
            static_cast<unsigned long long>(cid),
            static_cast<unsigned long long>(dispatch_generation));
          stale_generation = true;
        } else {
          std::shared_ptr<ServiceT::Response> resp;
          try {
            resp = future.get();
          } catch (const std::exception & ex) {
            RCLCPP_WARN(
              this->get_logger(), "LSposUpdate response error: %s", ex.what());
            unresolved = true;
          }
          if (!unresolved) {
            if (!resp) {
              unresolved = true;
            } else {
              RCLCPP_INFO(
                this->get_logger(),
                "LSposUpdate %s/%s correction_id=%llu accepted=%d applied=%d "
                "outcome=%s applied_delta=[%.4f %.4f %.4f]",
                type.c_str(),
                g6d::source_name(source),
                static_cast<unsigned long long>(cid),
                resp->accepted ? 1 : 0,
                resp->applied ? 1 : 0,
                resp->outcome.c_str(),
                resp->applied_delta_x, resp->applied_delta_y, resp->applied_delta_z);
              if (resp->outcome == "apply_pending") {
                unresolved = true;
              }
              applied = resp->applied;
              send_legacy_c = g6_coord::use_legacy_control_c(
                  used_coordination, type == "close_loop", applied) &&
                notify_control_on_apply &&
                run_ctrl_.should_emit_control_for(source);
            }
          }
        }
      }
      if (stale_generation) {
        if (used_coordination) {
          Release_or_abort_control(provenance, type, false, true, false);
          std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
          g6b_fb::clear_txn(pending_corr_txn_);
          Cancel_feedback_timeout_timer();
        }
        return;
      }
      if (!used_coordination) {
        if (unresolved) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Correction outcome unresolved (correction_id=%llu)",
            static_cast<unsigned long long>(cid));
          return;
        }
        if (send_legacy_c) {
          Send_control_loop_closed_signal();
        }
        return;
      }

      bool ready_release = false;
      bool arm_timer = false;
      g6b_fb::TxnPhase phase = g6b_fb::TxnPhase::Idle;
      {
        std::lock_guard<std::mutex> lock(mutex_corr_barrier_);
        if (pending_corr_txn_.correction_id != cid) {
          RCLCPP_WARN(
            this->get_logger(),
            "LSposUpdate response for stale/foreign correction_id=%llu "
            "(pending=%llu)",
            static_cast<unsigned long long>(cid),
            static_cast<unsigned long long>(pending_corr_txn_.correction_id));
          return;
        }
        ready_release = g6b_fb::on_ls_outcome(
          pending_corr_txn_, applied, unresolved);
        phase = pending_corr_txn_.phase;
        arm_timer = (phase == g6b_fb::TxnPhase::AwaitingFeedback);
        if (unresolved) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Correction outcome unresolved (correction_id=%llu); "
            "holding coordination pause (no auto-resume)",
            static_cast<unsigned long long>(cid));
        }
      }
      if (ready_release || phase == g6b_fb::TxnPhase::NotApplied ||
        phase == g6b_fb::TxnPhase::UnresolvedHold)
      {
        Complete_coordinated_txn_locked(
          provenance, type, dispatch_generation, phase);
        return;
      }
      if (arm_timer) {
        Arm_feedback_timeout_timer(cid);
      }
    };

  (void)client_local_slam_pos_update_->async_send_request(request, callback);
}

void Gslam::Send_control_loop_closed_signal()
{
  if (!client_control_run_ || !client_control_run_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "control_run_service unavailable; dropping loop-closed signal");
    return;
  }
  auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
  request->cmd = 'c';
  (void)client_control_run_->async_send_request(request);
}

void Gslam::Publish_global_loop_committed(
  const CorrectionProvenance & provenance,
  uint64_t map_revision,
  const std::string & source,
  const arma::vec::fixed<3> & committed_camera,
  bool match_ba_refine_ran)
{
  if (!pub_global_loop_committed_) {
    return;
  }
  interfaces::msg::GlobalLoopCommitted msg;
  msg.session_id = slam_identity::to_msg(provenance.session_id);
  msg.epoch_id = provenance.epoch_id;
  msg.keyframe_id = provenance.keyframe_id;
  msg.map_revision = map_revision;
  msg.source = source;
  msg.committed_camera_x = committed_camera[0];
  msg.committed_camera_y = committed_camera[1];
  msg.committed_camera_z = committed_camera[2];
  msg.match_ba_refine_ran = match_ba_refine_ran;
  pub_global_loop_committed_->publish(msg);
  RCLCPP_INFO(
    this->get_logger(),
    "GlobalLoopCommitted source=%s map_revision=%llu keyframe_id=%llu "
    "camera=[%.3f %.3f %.3f] match_ba_refine=%d "
    "(informational — does not imply Local/Control feedback)",
    source.c_str(),
    static_cast<unsigned long long>(map_revision),
    static_cast<unsigned long long>(provenance.keyframe_id),
    committed_camera[0], committed_camera[1], committed_camera[2],
    match_ba_refine_ran ? 1 : 0);
}

void Gslam::Publish_gmap_data()
{
  // Legacy custom Gmap (internal camera poses; no D). Retained during transition.
  // Prefer ~/global_keyframes + ~/global_landmarks for RViz2/Plot2.
  auto message = interfaces::msg::Gmap();

  for (size_t i = 0; i < Gmap.KeyFDATA.size(); i++) {
    arma::mat::fixed<3, 3> Rn2c = Gmap.KeyFDATA[i].Rn2c;
    arma::vec::fixed<3> t_c2n = Gmap.KeyFDATA[i].t_c2n;

    arma::vec::fixed<4> q_n2c;
    Ra2b_TO_Quat_a2b_arma(Rn2c, q_n2c);

    geometry_msgs::msg::Transform t_n2c;
    t_n2c.translation.x = t_c2n(0);
    t_n2c.translation.y = t_c2n(1);
    t_n2c.translation.z = t_c2n(2);
    t_n2c.rotation.w = q_n2c(0);
    t_n2c.rotation.x = q_n2c(1);
    t_n2c.rotation.y = q_n2c(2);
    t_n2c.rotation.z = q_n2c(3);
    message.n2c.push_back(t_n2c);
  }

  int j = PAR.GM_factor_anchors_sent_to_plot;
  if (Gmap.AnchorsDATA.size() > 100) {
    j = 10;
  }
  if (j < 1) {
    j = 1;
  }

  for (size_t i = 0; i < Gmap.AnchorsDATA.size(); i = i + static_cast<size_t>(j)) {
    geometry_msgs::msg::Point p;
    p.x = Gmap.AnchorsDATA[i].AnchorState[0];
    p.y = Gmap.AnchorsDATA[i].AnchorState[1];
    p.z = Gmap.AnchorsDATA[i].AnchorState[2];
    message.gmap_anchors.push_back(p);
  }

  pub_gmap_data_->publish(message);
}

bool Gslam::Capture_map_snapshot_locked(
  g4_viz::MapSnapshot & snap, uint64_t generation)
{
  // Caller holds mutex_run_ctrl_ + mutex_get_gm.
  if (!run_ctrl_.results_belong_to_generation(generation)) {
    return false;
  }
  snap = g4_viz::MapSnapshot{};
  snap.generation = generation;
  snap.epoch_id = active_ref_.epoch_id;
  snap.keyframes.reserve(Gmap.KeyFDATA.size());
  for (const auto & kf : Gmap.KeyFDATA) {
    g4_viz::SnapshotKeyframe sk;
    // BA/CL revise camera pose (t_c2n, Rn2c) but leave r_N/Rn2r stale.
    // Reconstruct body from the committed camera pose + stored extrinsics:
    //   Rn2c = Rr2c * Rn2r  ⇒  Rn2r = Rr2c^T * Rn2c
    //   t_c2n = r_N + Rn2r^T * t_c2r  ⇒  r_N = t_c2n - Rn2r^T * t_c2r
    const arma::mat::fixed<3, 3> Rn2r = kf.Rr2c.t() * kf.Rn2c;
    const arma::vec::fixed<3> r_N = kf.t_c2n - Rn2r.t() * kf.t_c2r;
    sk.r_N[0] = r_N(0);
    sk.r_N[1] = r_N(1);
    sk.r_N[2] = r_N(2);
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        sk.Rn2r_cm[c * 3 + r] = Rn2r(r, c);
      }
    }
    sk.source_stamp_valid = kf.source_stamp_valid;
    if (kf.source_stamp_valid) {
      sk.stamp_sec = static_cast<int32_t>(kf.source_time_ms / 1000L);
      sk.stamp_nanosec =
        static_cast<uint32_t>((kf.source_time_ms % 1000L) * 1000000L);
    }
    sk.keyframe_id = kf.keyframe_id;
    snap.keyframes.push_back(sk);
  }
  snap.landmarks.reserve(Gmap.AnchorsDATA.size());
  for (const auto & a : Gmap.AnchorsDATA) {
    g4_viz::SnapshotLandmark lm;
    lm.x = a.AnchorState(0);
    lm.y = a.AnchorState(1);
    lm.z = a.AnchorState(2);
    snap.landmarks.push_back(lm);
  }
  return true;
}

void Gslam::Publish_standard_viz(uint64_t generation)
{
  g4_viz::MapSnapshot snap;
  {
    std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
    std::lock_guard<std::mutex> lock_gm(mutex_get_gm);
    if (!Capture_map_snapshot_locked(snap, generation)) {
      return;
    }
  }

  std_msgs::msg::Header header;
  header.frame_id = public_frame_id_;
  header.stamp = this->get_clock()->now();

  const auto path = g4_viz::make_keyframe_path(
    header, snap.keyframes, path_max_poses_);
  const auto cloud = g4_viz::make_landmark_cloud(
    header, snap.landmarks, landmark_cloud_max_points_);

  {
    std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
    // Drop if a reset/new-reference emptied the map after the snapshot.
    if (!run_ctrl_.results_belong_to_generation(generation)) {
      return;
    }
  }

  pub_global_keyframes_->publish(path);
  pub_global_landmarks_->publish(cloud);
}

void Gslam::Publish_empty_standard_viz(uint64_t generation)
{
  std_msgs::msg::Header header;
  header.frame_id = public_frame_id_;
  header.stamp = this->get_clock()->now();
  const auto path = g4_viz::make_empty_path(header);
  const auto cloud = g4_viz::make_empty_cloud(header);

  {
    std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
    // Only latch empty outputs that still belong to this reset generation.
    if (!run_ctrl_.results_belong_to_generation(generation)) {
      return;
    }
  }
  pub_global_keyframes_->publish(path);
  pub_global_landmarks_->publish(cloud);
}

bool Gslam::try_admit_and_convert_kf(
  const interfaces::msg::Kf & msg, KEYFRAME & kf_out, const char * stream_name)
{
  {
    std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
    if (!run_ctrl_.should_accept_keyframe()) {
      return false;
    }
    const auto admit = admit_keyframe(
      active_ref_,
      slam_identity::from_msg(msg.session_id),
      msg.epoch_id,
      msg.keyframe_id,
      msg.stream,
      msg.source_stamp_valid,
      accept_missing_source_stamp_,
      static_cast<int>(msg.img.height),
      static_cast<int>(msg.img.width),
      PAR.Mono_cam_img_rows,
      PAR.Mono_cam_img_cols,
      seen_keyframe_ids_);
    if (!admit.accepted) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Gslam drop %s KF id=%llu: %s",
        stream_name,
        static_cast<unsigned long long>(msg.keyframe_id),
        admit.reason.c_str());
      return false;
    }
  }

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg.img, sensor_msgs::image_encodings::MONO8);

  kf_out.frame = cv_ptr->image;
  arma::vec::fixed<3> t_c2r;
  t_c2r(0) = msg.r2c.translation.x;
  t_c2r(1) = msg.r2c.translation.y;
  t_c2r(2) = msg.r2c.translation.z;
  kf_out.t_c2r = t_c2r;

  double quat[4];
  quat[0] = msg.r2c.rotation.w;
  quat[1] = msg.r2c.rotation.x;
  quat[2] = msg.r2c.rotation.y;
  quat[3] = msg.r2c.rotation.z;
  double Rr2c_a[9];
  quat2R_col_major(quat, Rr2c_a);
  arma::mat Rr2c(Rr2c_a, 3, 3);
  kf_out.Rr2c = Rr2c;

  arma::vec::fixed<3> r_N;
  r_N(0) = msg.n2r.translation.x;
  r_N(1) = msg.n2r.translation.y;
  r_N(2) = msg.n2r.translation.z;
  kf_out.r_N = r_N;

  double quat2[4];
  quat2[0] = msg.n2r.rotation.w;
  quat2[1] = msg.n2r.rotation.x;
  quat2[2] = msg.n2r.rotation.y;
  quat2[3] = msg.n2r.rotation.z;
  double Rn2r_a[9];
  quat2R_col_major(quat2, Rn2r_a);
  arma::mat Rn2r(Rn2r_a, 3, 3);
  kf_out.Rn2r = Rn2r;

  kf_out.Rn2c = Rr2c * Rn2r;
  kf_out.t_c2n = r_N + Rn2r.t() * t_c2r;
  kf_out.radius_uncertainty = msg.radius_uncertainty;

  // Preserve identity captured by Local SLAM at selection time.
  kf_out.session_id = slam_identity::from_msg(msg.session_id);
  kf_out.epoch_id = msg.epoch_id;
  kf_out.keyframe_id = msg.keyframe_id;
  kf_out.stream = msg.stream;
  kf_out.source_stamp_valid = msg.source_stamp_valid;
  if (msg.source_stamp_valid) {
    kf_out.source_time_ms =
      static_cast<long int>(msg.header.stamp.sec) * 1000L +
      static_cast<long int>(msg.header.stamp.nanosec) / 1000000L;
  } else {
    kf_out.source_time_ms = 0;
  }
  return true;
}

void Gslam::EstimatorReference_callback(
  const interfaces::msg::EstimatorReference & msg)
{
  const auto sid = slam_identity::from_msg(msg.session_id);
  RefUpdateAction action = RefUpdateAction::None;
  {
    std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
    action = classify_reference_update(active_ref_, msg.valid, sid, msg.epoch_id);
    if (action == RefUpdateAction::RejectStaleSession) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Gslam ignoring stale/conflict EstimatorReference session=%s",
        slam_identity::to_string(sid).c_str());
      return;
    }
    if (action == RefUpdateAction::None) {
      return;
    }
    apply_reference_update(active_ref_, action, msg.valid, sid, msg.epoch_id);
  }
  // clear_for_reference_change takes mutex_run_ctrl_ — must not hold it here.

  if (action == RefUpdateAction::Invalidate ||
    action == RefUpdateAction::IdentityChange)
  {
    clear_for_reference_change(
      action == RefUpdateAction::Invalidate ?
        "EstimatorReference invalid" :
        "EstimatorReference session/epoch change");
  } else if (action == RefUpdateAction::MetadataOnly) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Gslam EstimatorReference metadata-only update (same session/epoch)");
  }
}

void Gslam::Kf_cl_callback(const interfaces::msg::Kf & msg)
{
  KEYFRAME kf;
  if (!try_admit_and_convert_kf(msg, kf, "close_loop")) {
    return;
  }
  // Capture uses the admitted LS snapshot (same object), before any Gmap mutation.
  if (home_capture_active_) {
    std::lock_guard<std::mutex> lock_q(mutex_home_capture_q_);
    home_capture_kf_q_.push_back(kf);
  }
  std::lock_guard<std::mutex> lock(mutex_rx_kf_cl);
  kf_cl_last = kf;
  new_kf_cl = true;
}

void Gslam::Kf_callback(const interfaces::msg::Kf & msg)
{
  KEYFRAME kf;
  if (!try_admit_and_convert_kf(msg, kf, "map")) {
    return;
  }
  if (home_capture_active_) {
    std::lock_guard<std::mutex> lock_q(mutex_home_capture_q_);
    home_capture_kf_q_.push_back(kf);
  }
  std::lock_guard<std::mutex> lock(mutex_rx_kf);
  Kf_buffer.push_back(kf);
}

void Gslam::Handle_enable_service(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  {
    // Lock order: run_ctrl → kf → kf_cl → gm
    std::lock_guard<std::mutex> lock_ctrl(mutex_run_ctrl_);
    run_ctrl_.set_processing_enabled(request->data);
    if (!request->data) {
      std::lock_guard<std::mutex> lock_kf(mutex_rx_kf);
      std::lock_guard<std::mutex> lock_cl(mutex_rx_kf_cl);
      std::lock_guard<std::mutex> lock_gm(mutex_get_gm);
      clear_pending_keyframes_locked();
    }
  }
  response->success = true;
  response->message = request->data ?
    "globalslam processing enabled (in-flight steps may still finish; "
    "corrections gated by Apply_global_corrections; already-dispatched "
    "async requests cannot be recalled)" :
    "globalslam processing disabled; pending KF dropped "
    "(does not promise worker quiescence; already-dispatched async "
    "requests cannot be recalled)";
  RCLCPP_INFO(this->get_logger(), "Gslam ~/enable %s", response->message.c_str());
}

void Gslam::Handle_reset_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  request_reset();
  response->success = true;
  response->message = "globalslam reset requested";
}

void Gslam::Handle_globalslam_run_service(
  const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,
  std::shared_ptr<interfaces::srv::SimpleServ::Response> response)
{
  if (request->cmd == '0') {
    std::lock_guard<std::mutex> lock_g(mutex_log_gmap);
    gmap_get_log_flag = true;
    std::lock_guard<std::mutex> lock_c(mutex_log_cloop);
    cloop_get_log_flag = true;
  }
  if (request->cmd == 'i') {
    request_reset();
  }
  if (request->cmd == 'b') {
    // Begin RecHome home-tag position capture (before exploration).
    {
      std::lock_guard<std::mutex> lock(mutex_rechome);
      rechome_begin_flag_ = true;
      home_capture_active_ = true;  // enqueue admitted KFs immediately
      home_capture_finalize_done_ = false;
      home_capture_finalize_result_ = false;
      home_capture_finalize_pending_ = false;
      home_capture_invalidate_ = false;
    }
    response->response = true;
  } else if (request->cmd == 'u') {
    // End RecHome: finalize capture + stamp n_kf_home.
    {
      std::lock_guard<std::mutex> lock(mutex_rechome);
      rechome_flag = true;
      home_capture_finalize_done_ = false;
    }
    // Bounded wait for CLOOP worker to finalize (drain + estimate).
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool done = false;
    bool ok = false;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_rechome);
        done = home_capture_finalize_done_;
        ok = home_capture_finalize_result_;
      }
      if (done) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    response->response = done && ok;
    if (!done) {
      RCLCPP_WARN(
        this->get_logger(),
        "RecHome finalize timed out waiting for capture worker");
    } else if (!ok) {
      RCLCPP_WARN(
        this->get_logger(),
        "RecHome finalize completed without a ready home-tag reference");
    }
  } else if (request->cmd == 'c') {
    std::lock_guard<std::mutex> lock(mutex_activate_CL);
    activate_CL_flag = true;
    response->response = true;
  } else {
    response->response = true;
  }
}

void Gslam::log_data_g(STOREG & data)
{
  cout << "----> Global Map stats: <----" << endl;
  cout << "Total number of keyframes: " << data.n_kf << endl;
  cout << "Total number of initialized Anchors: " << data.n_init_anchors << endl;
  cout << "Total number of deleted Anchors: " << data.n_delete_anchors << endl;
  cout << "Map size (number of anchors): " << data.n_init_anchors - data.n_delete_anchors << endl;
  cout << "Total computation time: " << data.total_comp_time << endl;

  double mean_anchors, std_anchors, sum_anchors;
  mean_std(data.n_anchors_per_step.second, mean_anchors, std_anchors, sum_anchors);
  cout << "Mean Anchors per step: " << mean_anchors << "  Std. Anchors per step: " << std_anchors <<
    endl;

  double mean_time, std_time, sum_time;
  mean_std(data.time_per_step.second, mean_time, std_time, sum_time);
  cout << "Mean Computation time per step: " << mean_time <<
    "  Std. Computation time per frame: " << std_time << endl;

  std::ofstream myfile("log_gmap");
  if (myfile.is_open()) {
    myfile << "Total number of keyframes: " << data.n_kf << endl;
    myfile << "Total number of initialized Anchors: " << data.n_init_anchors << endl;
    myfile << "Total number of deleted Anchors: " << data.n_delete_anchors << endl;
    myfile << "Map size (number of anchors): " << data.n_init_anchors - data.n_delete_anchors <<
      endl;
    myfile << "Total computation time: " << data.total_comp_time << endl;
    myfile << "Mean Anchors per step: " << mean_anchors << "  Std. Anchors per step: " <<
      std_anchors << endl;
    myfile << "Mean Computation time per step: " << mean_time <<
      "  Std. Computation time per step: " << std_time << endl;
    myfile.close();
  } else {
    std::cout << "Unable to create or open the file.\n";
  }
  log_data_to_file("log_gmap_anchors_per_step", data.n_anchors_per_step);
  log_data_to_file("log_gmap_time_per_step", data.time_per_step);
  log_data_to_file("log_gmap_kf_per_step", data.n_kf_per_step);
}

void Gslam::log_data_c(STOREC & data)
{
  cout << "----> Closing loop stats: <----" << endl;
  cout << "Total computation time: " << data.total_comp_time << endl;

  double mean_time, std_time, sum_time;
  mean_std(data.time_search_per_step.second, mean_time, std_time, sum_time);
  cout << "Mean Computation time per step: " << mean_time <<
    "  Std. Computation time per step: " << std_time << endl;

  for (size_t i = 0; i < data.cloop_stats.size(); i++) {
    cout << "-- Closing loop stats at step: " << data.cloop_stats[i].step << endl;
    cout << "   Number of intents before CL: " << data.cloop_stats[i].n_loop_closing_intents <<
      endl;
    cout << "   XY Error correction y pose (m): " << data.cloop_stats[i].error_correction << endl;
    cout << "   Mean reprojection error (pixel): " << data.cloop_stats[i].reprojection_error <<
      endl;
    cout << "   Distance traveled (m): " << data.cloop_stats[i].distance_traveled << endl;
  }

  std::ofstream myfile("log_cloop");
  if (myfile.is_open()) {
    myfile << "Total search computation time: " << data.total_comp_time << endl;
    myfile << "Mean Computation search time per step: " << mean_time <<
      "  Std. Computation search time per step: " << std_time << endl;
    for (size_t i = 0; i < data.cloop_stats.size(); i++) {
      myfile << "-- Closing loop stats at step: " << data.cloop_stats[i].step << endl;
      myfile << "   Number of intents before CL: " << data.cloop_stats[i].n_loop_closing_intents <<
        endl;
      myfile << "   XY Error correction y pose (m): " << data.cloop_stats[i].error_correction <<
        endl;
      myfile << "   Mean reprojection error (pixel): " << data.cloop_stats[i].reprojection_error <<
        endl;
      myfile << "   Distance traveled (m): " << data.cloop_stats[i].distance_traveled << endl;
    }
    myfile.close();
  } else {
    std::cout << "Unable to create or open the file.\n";
  }
  log_data_to_file("log_cloop_time_per_step", data.time_search_per_step);
}

void Gslam::log_data_to_file(
  string file_name, std::pair<std::vector<double>, std::vector<double>> & data)
{
  std::ofstream myfile(file_name);
  if (myfile.is_open()) {
    for (size_t i = 0; i < data.first.size(); i++) {
      myfile << data.first[i] << "," << data.second[i] << endl;
    }
    myfile.close();
  } else {
    std::cout << "Unable to create or open the file.\n";
  }
}

void Gslam::mean_std(
  std::vector<double> & data, double & mean, double & std, double & sum)
{
  sum = 0;
  for (size_t i = 0; i < data.size(); i++) {
    sum = sum + data[i];
  }
  mean = data.empty() ? 0.0 : sum / static_cast<double>(data.size());

  double sum_sq = 0;
  for (size_t i = 0; i < data.size(); i++) {
    sum_sq = sum_sq + pow(data[i] - mean, 2);
  }
  std = data.empty() ? 0.0 : sqrt(sum_sq / static_cast<double>(data.size()));
}

}  // namespace globalslam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(globalslam::Gslam)
