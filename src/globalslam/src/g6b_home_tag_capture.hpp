#ifndef GLOBALSLAM_G6B_HOME_TAG_CAPTURE_HPP_
#define GLOBALSLAM_G6B_HOME_TAG_CAPTURE_HPP_

/**
 * G6b.3: epoch-owned home-tag POSITION reference from original Local EKF snapshots.
 *
 * Capture uses immutable RecordingObservation copies (pose at LS selection time).
 * Globally optimized KEYFRAME.t_c2n / Rn2c must never feed this estimator.
 *
 * Geometry (Rn2c = R_C←N, tvec = t_H_C = OpenCV marker origin in C):
 *   p_H_N = p_C_N + Rn2cᵀ · tvec
 *   p_C_N = p_H_N − Rn2cᵀ · tvec
 *
 * Robust finalization: per-axis median center → Euclidean outlier reject →
 * mean of inliers. Requires min inliers and min inlier fraction; max spread.
 *
 * States: Unrecorded → Recording → Ready | Failed
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "../../common/slam_identity.hpp"
#include "g6b_home_tag_geometry.hpp"

namespace globalslam
{
namespace g6b_home_tag
{

enum class CaptureState : uint8_t
{
  Unrecorded = 0,
  Recording = 1,
  Ready = 2,
  Failed = 3,
};

inline const char * capture_state_name(CaptureState s)
{
  switch (s) {
    case CaptureState::Unrecorded: return "unrecorded";
    case CaptureState::Recording: return "recording";
    case CaptureState::Ready: return "ready";
    case CaptureState::Failed: return "failed";
  }
  return "unknown";
}

struct CaptureParams
{
  int home_tag_id{23};
  double tag_size_m{1.0};
  int min_inliers{3};
  double min_inlier_fraction{0.5};  // inliers / unique candidates
  double max_spread_m{1.5};         // ||p - median|| reject threshold [m]
  int max_samples{40};              // bound storage of unique candidates
  double gate_px{90.0};             // marker-center vs principal point
};

/** Immutable LS snapshot observation (one unique keyframe_id). */
struct RecordingObservation
{
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  uint64_t keyframe_id{0};
  long int source_time_ms{0};
  bool source_stamp_valid{false};
  int tag_id{-1};
  double t_c2n[3]{0, 0, 0};   // original EKF camera position in N
  double Rn2c_rm[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};  // original R_C←N row-major
  double tvec[3]{0, 0, 0};    // OpenCV t_H_C
  double p_H_N[3]{0, 0, 0};   // candidate tag position
  double d_px{0};
};

struct CaptureDiagnostics
{
  int n_candidates{0};
  int n_inliers{0};
  double spread_m{0};       // max ||inlier - mean|| after final mean
  double median_spread_m{0};  // max ||candidate - median|| before reject
  std::string fail_reason;
};

struct CaptureParamsRuntime : CaptureParams {};

inline void tag_position_from_camera(
  double t_c2n_x, double t_c2n_y, double t_c2n_z,
  double tvec_x, double tvec_y, double tvec_z,
  const double Rn2c_rm[9],
  double & p_H_x, double & p_H_y, double & p_H_z)
{
  // Rn2cᵀ * tvec
  const double rx =
    Rn2c_rm[0] * tvec_x + Rn2c_rm[3] * tvec_y + Rn2c_rm[6] * tvec_z;
  const double ry =
    Rn2c_rm[1] * tvec_x + Rn2c_rm[4] * tvec_y + Rn2c_rm[7] * tvec_z;
  const double rz =
    Rn2c_rm[2] * tvec_x + Rn2c_rm[5] * tvec_y + Rn2c_rm[8] * tvec_z;
  p_H_x = t_c2n_x + rx;
  p_H_y = t_c2n_y + ry;
  p_H_z = t_c2n_z + rz;
}

inline void camera_position_from_tag(
  double p_H_x, double p_H_y, double p_H_z,
  double tvec_x, double tvec_y, double tvec_z,
  const double Rn2c_rm[9],
  double & t_c2n_x, double & t_c2n_y, double & t_c2n_z)
{
  const double rx =
    Rn2c_rm[0] * tvec_x + Rn2c_rm[3] * tvec_y + Rn2c_rm[6] * tvec_z;
  const double ry =
    Rn2c_rm[1] * tvec_x + Rn2c_rm[4] * tvec_y + Rn2c_rm[7] * tvec_z;
  const double rz =
    Rn2c_rm[2] * tvec_x + Rn2c_rm[5] * tvec_y + Rn2c_rm[8] * tvec_z;
  t_c2n_x = p_H_x - rx;
  t_c2n_y = p_H_y - ry;
  t_c2n_z = p_H_z - rz;
}

inline double dist3(
  double ax, double ay, double az, double bx, double by, double bz)
{
  const double dx = ax - bx, dy = ay - by, dz = az - bz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double median_of(std::vector<double> v)
{
  if (v.empty()) {
    return 0.0;
  }
  const size_t n = v.size();
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n / 2), v.end());
  if (n % 2 == 1) {
    return v[n / 2];
  }
  const double hi = v[n / 2];
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n / 2 - 1), v.end());
  return 0.5 * (hi + v[n / 2 - 1]);
}

/**
 * Robust position from candidates: median center → reject by max_spread_m →
 * mean of inliers. Does not treat dispersion as measurement covariance.
 */
inline bool estimate_robust_tag_position(
  const std::vector<RecordingObservation> & samples,
  const CaptureParams & params,
  double p_H_out[3],
  CaptureDiagnostics & diag)
{
  diag = CaptureDiagnostics{};
  diag.n_candidates = static_cast<int>(samples.size());
  if (samples.empty()) {
    diag.fail_reason = "no_candidates";
    return false;
  }

  std::vector<double> xs, ys, zs;
  xs.reserve(samples.size());
  ys.reserve(samples.size());
  zs.reserve(samples.size());
  for (const auto & s : samples) {
    xs.push_back(s.p_H_N[0]);
    ys.push_back(s.p_H_N[1]);
    zs.push_back(s.p_H_N[2]);
  }
  const double cx = median_of(xs);
  const double cy = median_of(ys);
  const double cz = median_of(zs);

  double max_to_median = 0.0;
  std::vector<size_t> inlier_idx;
  inlier_idx.reserve(samples.size());
  for (size_t i = 0; i < samples.size(); ++i) {
    const double d = dist3(
      samples[i].p_H_N[0], samples[i].p_H_N[1], samples[i].p_H_N[2],
      cx, cy, cz);
    max_to_median = std::max(max_to_median, d);
    if (d <= params.max_spread_m) {
      inlier_idx.push_back(i);
    }
  }
  diag.median_spread_m = max_to_median;
  diag.n_inliers = static_cast<int>(inlier_idx.size());

  const double frac =
    static_cast<double>(diag.n_inliers) /
    static_cast<double>(diag.n_candidates);
  if (diag.n_inliers < params.min_inliers) {
    diag.fail_reason = "insufficient_inliers";
    return false;
  }
  if (frac + 1e-15 < params.min_inlier_fraction) {
    diag.fail_reason = "insufficient_inlier_fraction";
    return false;
  }

  double mx = 0, my = 0, mz = 0;
  for (size_t i : inlier_idx) {
    mx += samples[i].p_H_N[0];
    my += samples[i].p_H_N[1];
    mz += samples[i].p_H_N[2];
  }
  const double inv = 1.0 / static_cast<double>(inlier_idx.size());
  p_H_out[0] = mx * inv;
  p_H_out[1] = my * inv;
  p_H_out[2] = mz * inv;

  double max_to_mean = 0.0;
  for (size_t i : inlier_idx) {
    max_to_mean = std::max(
      max_to_mean,
      dist3(
        samples[i].p_H_N[0], samples[i].p_H_N[1], samples[i].p_H_N[2],
        p_H_out[0], p_H_out[1], p_H_out[2]));
  }
  diag.spread_m = max_to_mean;
  if (diag.spread_m > params.max_spread_m) {
    diag.fail_reason = "excessive_final_spread";
    return false;
  }
  return true;
}

/**
 * Capture session controller (thread-safe usage requires external mutex).
 */
class HomeTagCaptureSession
{
public:
  CaptureState state() const { return state_; }
  const HomeTagReference & reference() const { return ref_; }
  const CaptureDiagnostics & diagnostics() const { return diag_; }
  bool is_recording() const { return state_ == CaptureState::Recording; }
  bool is_ready() const
  {
    return state_ == CaptureState::Ready && ref_.spatial_ready;
  }
  bool finalize_requested() const { return finalize_requested_; }

  void set_params(const CaptureParams & p) { params_ = p; }
  const CaptureParams & params() const { return params_; }

  void clear()
  {
    state_ = CaptureState::Unrecorded;
    ref_ = HomeTagReference{};
    diag_ = CaptureDiagnostics{};
    samples_.clear();
    seen_kf_ids_.clear();
    finalize_requested_ = false;
    cutoff_keyframe_id_ = 0;
    cutoff_source_time_ms_ = 0;
    max_seen_keyframe_id_ = 0;
    max_seen_source_time_ms_ = 0;
  }

  /** Start recording. Refuses silent replace of Ready (deliberate recapture). */
  bool begin(
    const slam_identity::UuidBytes & session_id,
    uint64_t epoch_id,
    std::string & reason)
  {
    if (state_ == CaptureState::Ready && ref_.spatial_ready) {
      reason = "already_ready_deliberate_recapture_required";
      return false;
    }
    clear();
    state_ = CaptureState::Recording;
    ref_.session_id = session_id;
    ref_.epoch_id = epoch_id;
    ref_.valid = true;  // lifecycle stamp while recording (not spatial_ready)
    ref_.spatial_ready = false;
    ref_.tag_id = params_.home_tag_id;
    reason.clear();
    return true;
  }

  void request_finalize()
  {
    if (state_ != CaptureState::Recording) {
      return;
    }
    finalize_requested_ = true;
    cutoff_keyframe_id_ = max_seen_keyframe_id_;
    cutoff_source_time_ms_ = max_seen_source_time_ms_;
  }

  /** Invalidate active recording (e.g. correction applied / reset). */
  void fail(const std::string & reason)
  {
    if (state_ == CaptureState::Ready) {
      // Frozen ready ref is not auto-cleared by correction; only reset/epoch.
      return;
    }
    diag_.fail_reason = reason;
    state_ = CaptureState::Failed;
    ref_.spatial_ready = false;
    // Keep lifecycle valid=false so return path withholds corrections.
    ref_.valid = false;
    finalize_requested_ = false;
    samples_.clear();
  }

  /**
   * Offer one LS snapshot observation (already deduped caller-side optional).
   * Returns true if accepted as a new unique candidate.
   */
  bool offer(const RecordingObservation & obs, std::string & reject)
  {
    reject.clear();
    if (state_ != CaptureState::Recording) {
      reject = "not_recording";
      return false;
    }
    if (!slam_identity::equal(obs.session_id, ref_.session_id) ||
      obs.epoch_id != ref_.epoch_id)
    {
      reject = "session_epoch_mismatch";
      return false;
    }
    if (obs.tag_id != params_.home_tag_id) {
      reject = "wrong_tag_id";
      return false;
    }
    if (finalize_requested_) {
      if (obs.keyframe_id > cutoff_keyframe_id_) {
        reject = "after_cutoff_id";
        return false;
      }
      if (obs.source_stamp_valid && cutoff_source_time_ms_ > 0 &&
        obs.source_time_ms > cutoff_source_time_ms_)
      {
        reject = "after_cutoff_time";
        return false;
      }
    }
    if (seen_kf_ids_.count(obs.keyframe_id) != 0) {
      reject = "duplicate_keyframe_id";
      return false;
    }
    if (static_cast<int>(samples_.size()) >= params_.max_samples) {
      reject = "max_samples";
      return false;
    }
    if (!is_finite3(obs.t_c2n[0], obs.t_c2n[1], obs.t_c2n[2]) ||
      !is_finite3(obs.tvec[0], obs.tvec[1], obs.tvec[2]) ||
      !is_finite3(obs.p_H_N[0], obs.p_H_N[1], obs.p_H_N[2]))
    {
      reject = "non_finite";
      return false;
    }
    if (!(obs.tvec[2] > 0.0)) {
      reject = "tag_not_in_front";
      return false;
    }
    if (!(obs.d_px < params_.gate_px)) {
      reject = "gate_px";
      return false;
    }

    seen_kf_ids_.insert(obs.keyframe_id);
    samples_.push_back(obs);
    max_seen_keyframe_id_ = std::max(max_seen_keyframe_id_, obs.keyframe_id);
    if (obs.source_stamp_valid) {
      max_seen_source_time_ms_ =
        std::max(max_seen_source_time_ms_, obs.source_time_ms);
    }
    return true;
  }

  /** Note a KF id/time even without a tag hit (for cutoff bookkeeping). */
  void note_keyframe_seen(uint64_t keyframe_id, long int source_time_ms, bool stamp_valid)
  {
    if (state_ != CaptureState::Recording) {
      return;
    }
    max_seen_keyframe_id_ = std::max(max_seen_keyframe_id_, keyframe_id);
    if (stamp_valid) {
      max_seen_source_time_ms_ =
        std::max(max_seen_source_time_ms_, source_time_ms);
    }
  }

  /**
   * Run robust estimator. Call after finalize requested and eligible queue drained.
   * @return true if Ready
   */
  bool finalize(int n_kf_home)
  {
    if (state_ == CaptureState::Ready) {
      return true;
    }
    if (state_ != CaptureState::Recording) {
      if (state_ != CaptureState::Failed) {
        fail("finalize_not_recording");
      }
      return false;
    }

    double p_H[3]{0, 0, 0};
    if (!estimate_robust_tag_position(samples_, params_, p_H, diag_)) {
      state_ = CaptureState::Failed;
      ref_.spatial_ready = false;
      ref_.valid = false;
      finalize_requested_ = false;
      return false;
    }

    ref_.p_H_N[0] = p_H[0];
    ref_.p_H_N[1] = p_H[1];
    ref_.p_H_N[2] = p_H[2];
    ref_.tag_id = params_.home_tag_id;
    ref_.n_kf_home = n_kf_home;
    ref_.n_inliers = diag_.n_inliers;
    ref_.n_candidates = diag_.n_candidates;
    ref_.spread_m = diag_.spread_m;
    ref_.spatial_ready = true;
    ref_.valid = true;
    state_ = CaptureState::Ready;
    finalize_requested_ = false;
    // Freeze: drop mutable samples (provenance retained on ref diagnostics).
    samples_.clear();
    return true;
  }

  int sample_count() const { return static_cast<int>(samples_.size()); }
  uint64_t cutoff_keyframe_id() const { return cutoff_keyframe_id_; }

private:
  CaptureState state_{CaptureState::Unrecorded};
  HomeTagReference ref_{};
  CaptureDiagnostics diag_{};
  CaptureParams params_{};
  std::vector<RecordingObservation> samples_;
  std::unordered_set<uint64_t> seen_kf_ids_;
  bool finalize_requested_{false};
  uint64_t cutoff_keyframe_id_{0};
  long int cutoff_source_time_ms_{0};
  uint64_t max_seen_keyframe_id_{0};
  long int max_seen_source_time_ms_{0};
};

}  // namespace g6b_home_tag
}  // namespace globalslam

#endif  // GLOBALSLAM_G6B_HOME_TAG_CAPTURE_HPP_
