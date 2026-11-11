#ifndef GLOBALSLAM_G6B_HOME_TAG_GEOMETRY_HPP_
#define GLOBALSLAM_G6B_HOME_TAG_GEOMETRY_HPP_

/**
 * Home-tag → internal camera pose (production conversion).
 *
 * ---------------------------------------------------------------------------
 * Transform contract (T_A←B maps coordinates expressed in B into A)
 * ---------------------------------------------------------------------------
 *
 * Frames:
 *   N — internal navigation (estimator; same axes as KEYFRAME.t_c2n / Rn2c)
 *   B — robot body
 *   C — camera optical (+Z forward; OpenCV / estimatePoseSingleMarkers)
 *   H — home-tag (ArUco; +Z out of marker plane; corners DICT_6X6_*)
 *
 * OpenCV aruco::estimatePoseSingleMarkers (this OpenCV API):
 *   tvec = marker-frame origin position in C  (= translation of T_C←H)
 *   rvec = Rodrigues(R_C←H)
 * so the detector supplies T_C←H.
 *
 * Recorded home reference (G6b.3): epoch-owned tag POSITION p_H_N captured
 * during RecHome from original Local EKF keyframe snapshots (not globally
 * optimized poses). RecHome also stamps session/epoch + n_kf_home.
 *
 * Geometry (Rn2c = R_C←N):
 *   p_H_N = p_C_N + Rn2cᵀ · tvec
 *   p_C_N = p_H_N − Rn2cᵀ · tvec
 *
 * Special case p_H_N = 0 recovers G6b.2 `t_c2n = −Rn2cᵀ · tvec`.
 *
 * Return path uses detecting-KF Rn2c (EKF attitude) — attitude error couples
 * into recovered position (audited). rvec is not a Pose_SLAM attitude factor.
 *
 * Pose_SLAM still uses BetweenFactor<Pose3> with measured R≡I (full 6-D
 * residual with finite σ_att — not an unconstrained translation-only factor).
 *
 * Public D=diag(1,−1,−1) is publish-only; never applied inside this chain.
 */

#include <cmath>
#include <cstdint>

#include "../../common/slam_identity.hpp"

namespace globalslam
{
namespace g6b_home_tag
{

/** Documented legacy CLOOP remap (regression / before evidence). */
inline void live_cloop_tvec_remap(
  double tvec_x, double tvec_y, double tvec_z,
  double & pos_x, double & pos_y, double & pos_z)
{
  pos_x = tvec_y;
  pos_y = -tvec_x;
  pos_z = tvec_z;
}

/** Documented Control GoHome remap (Y sign ≠ CLOOP). */
inline void live_control_tvec_remap(
  double tvec_x, double tvec_y, double tvec_z,
  double & pos_x, double & pos_y, double & pos_z)
{
  pos_x = tvec_y;
  pos_y = tvec_x;
  pos_z = tvec_z;
}

/**
 * OpenCV tvec → camera position in N given stored tag position p_H_N.
 * Rn2c_rm: row-major R_C←N.
 */
inline void camera_tvec_to_nav_camera_pos(
  double tvec_x, double tvec_y, double tvec_z,
  const double Rn2c_rm[9],
  double p_H_N_x, double p_H_N_y, double p_H_N_z,
  double & t_c2n_x, double & t_c2n_y, double & t_c2n_z)
{
  const double rx =
    Rn2c_rm[0] * tvec_x + Rn2c_rm[3] * tvec_y + Rn2c_rm[6] * tvec_z;
  const double ry =
    Rn2c_rm[1] * tvec_x + Rn2c_rm[4] * tvec_y + Rn2c_rm[7] * tvec_z;
  const double rz =
    Rn2c_rm[2] * tvec_x + Rn2c_rm[5] * tvec_y + Rn2c_rm[8] * tvec_z;
  t_c2n_x = p_H_N_x - rx;
  t_c2n_y = p_H_N_y - ry;
  t_c2n_z = p_H_N_z - rz;
}

/** G6b.2 special case: tag at N origin. */
inline void camera_tvec_to_nav_camera_pos_tag_at_origin(
  double tvec_x, double tvec_y, double tvec_z,
  const double Rn2c_rm[9],
  double & t_c2n_x, double & t_c2n_y, double & t_c2n_z)
{
  camera_tvec_to_nav_camera_pos(
    tvec_x, tvec_y, tvec_z, Rn2c_rm, 0.0, 0.0, 0.0,
    t_c2n_x, t_c2n_y, t_c2n_z);
}

/** Flatten arma-style row-major 3×3 (r00,r01,r02,r10,...). */
inline void flatten_Rn2c_row_major(
  double r00, double r01, double r02,
  double r10, double r11, double r12,
  double r20, double r21, double r22,
  double out_rm[9])
{
  out_rm[0] = r00; out_rm[1] = r01; out_rm[2] = r02;
  out_rm[3] = r10; out_rm[4] = r11; out_rm[5] = r12;
  out_rm[6] = r20; out_rm[7] = r21; out_rm[8] = r22;
}

inline bool nearly_equal(double a, double b, double tol = 1e-9)
{
  return std::fabs(a - b) <= tol;
}

inline bool is_finite3(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/**
 * Epoch-owned home-tag reference.
 * spatial_ready + p_H_N: frozen after successful RecHome capture.
 * valid: lifecycle (session/epoch stamp); may be true while recording.
 */
struct HomeTagReference
{
  bool valid{false};
  bool spatial_ready{false};
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  int n_kf_home{0};
  int tag_id{-1};
  double p_H_N[3]{0, 0, 0};
  int n_inliers{0};
  int n_candidates{0};
  double spread_m{0};
};

inline void clear_home_reference(HomeTagReference & ref)
{
  ref = HomeTagReference{};
}

inline void stamp_home_reference(
  HomeTagReference & ref,
  const slam_identity::UuidBytes & session_id,
  uint64_t epoch_id,
  int n_kf_home)
{
  ref.valid = true;
  ref.session_id = session_id;
  ref.epoch_id = epoch_id;
  ref.n_kf_home = n_kf_home;
}

enum class PoseReject : uint8_t
{
  Ok = 0,
  NoMarker,
  NonFinite,
  GateXyOrPixel,
  HomeRefMissing,
  SessionEpochMismatch,
  Rn2cInvalid,
  HomeRefNotReady,
};

inline const char * pose_reject_name(PoseReject r)
{
  switch (r) {
    case PoseReject::Ok: return "ok";
    case PoseReject::NoMarker: return "no_marker";
    case PoseReject::NonFinite: return "non_finite";
    case PoseReject::GateXyOrPixel: return "gate_xy_or_pixel";
    case PoseReject::HomeRefMissing: return "home_ref_missing";
    case PoseReject::SessionEpochMismatch: return "session_epoch_mismatch";
    case PoseReject::Rn2cInvalid: return "rn2c_invalid";
    case PoseReject::HomeRefNotReady: return "home_ref_not_ready";
  }
  return "unknown";
}

/**
 * Return-path conversion + gates.
 * Horizontal gate: √(Δx²+Δy²) of (p_C − p_H) ≡ horizontal ||Rn2cᵀ tvec||.
 * Requires spatial_ready (recorded p_H_N); no zero-origin fallback.
 */
inline PoseReject convert_home_tag_tvec_to_nav_camera_pos(
  double tvec_x, double tvec_y, double tvec_z,
  const double Rn2c_rm[9],
  double gate_xy_m,
  double gate_px, double d_px,
  bool home_ref_valid,
  bool home_ref_spatial_ready,
  bool session_epoch_ok,
  double p_H_N_x, double p_H_N_y, double p_H_N_z,
  double & t_c2n_x, double & t_c2n_y, double & t_c2n_z)
{
  if (!home_ref_valid) {
    return PoseReject::HomeRefMissing;
  }
  if (!home_ref_spatial_ready) {
    return PoseReject::HomeRefNotReady;
  }
  if (!session_epoch_ok) {
    return PoseReject::SessionEpochMismatch;
  }
  if (!is_finite3(tvec_x, tvec_y, tvec_z) ||
    !is_finite3(p_H_N_x, p_H_N_y, p_H_N_z))
  {
    return PoseReject::NonFinite;
  }
  if (!is_finite3(Rn2c_rm[0], Rn2c_rm[1], Rn2c_rm[2]) ||
    !is_finite3(Rn2c_rm[3], Rn2c_rm[4], Rn2c_rm[5]) ||
    !is_finite3(Rn2c_rm[6], Rn2c_rm[7], Rn2c_rm[8]))
  {
    return PoseReject::Rn2cInvalid;
  }
  camera_tvec_to_nav_camera_pos(
    tvec_x, tvec_y, tvec_z, Rn2c_rm,
    p_H_N_x, p_H_N_y, p_H_N_z,
    t_c2n_x, t_c2n_y, t_c2n_z);
  if (!is_finite3(t_c2n_x, t_c2n_y, t_c2n_z)) {
    return PoseReject::NonFinite;
  }
  const double dx = t_c2n_x - p_H_N_x;
  const double dy = t_c2n_y - p_H_N_y;
  const double d_xy_nav = std::sqrt(dx * dx + dy * dy);
  if (!(d_xy_nav < gate_xy_m && d_px < gate_px)) {
    return PoseReject::GateXyOrPixel;
  }
  return PoseReject::Ok;
}

}  // namespace g6b_home_tag
}  // namespace globalslam

#endif  // GLOBALSLAM_G6B_HOME_TAG_GEOMETRY_HPP_
