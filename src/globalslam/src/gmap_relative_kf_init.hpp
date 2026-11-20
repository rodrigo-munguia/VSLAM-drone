/**
 * GSLAM-BA-D6 / GSLAM-KF-Z-A2: keyframe pose initialization helpers.
 *
 * Camera pose T = (R=Rc2n=Rn2c^T, t=t_c2n) in nav frame.
 * Relative EKF motion composed onto the latest committed GlobalSLAM pose:
 *
 *   T_GS_i = T_GS_{i-1} * inv(T_EKF_{i-1}) * T_EKF_i
 *
 * T_EKF_* are absolute EKF poses at admission (never BA-updated).
 * T_GS_{i-1} is KeyFDATA[i-1] after any accepted BA commits.
 * Extrinsics (Rr2c, t_c2r) are left unchanged; body r_N/Rn2r are re-derived.
 *
 * Relative mode default ON (CONFIG-A1). Absolute admit:
 *   Init_relative_kf_pose_enable:=false or GSLAM_INIT_RELATIVE_KF_POSE=0.
 *
 * GSLAM-KF-Z-A5 hybrid (production default ON):
 *   Init_hybrid_kf_z_absolute_enable:=true with relative ON
 *   → relative SE(3) for R and XY; Z = T_EKF_curr.t_c2n.z (shared nav frame).
 * Full-relative: Init_hybrid_kf_z_absolute_enable:=false or
 *   GSLAM_INIT_HYBRID_KF_Z_ABS=0.
 * Hybrid has no effect when relative is OFF.
 * Z is anchored only at KF admission; Global BA may still modify Z afterward.
 * No BA / Pose_SLAM / LocalSLAM algorithm changes.
 */
#ifndef GMAP_RELATIVE_KF_INIT_HPP
#define GMAP_RELATIVE_KF_INIT_HPP

#include <cstdlib>
#include <cstdio>
#include <armadillo>

#include "../../common/keyframe_types.hpp"

namespace gmap_relative_kf_init
{

/** Matches declare_parameter / experiment YAML default. */
inline constexpr bool kInitRelativeKfPoseDefault = true;

/** Hybrid Z-absolute admit; production default ON (KF-Z-A5). */
inline constexpr bool kInitHybridKfZAbsoluteDefault = true;

/**
 * Resolve Init_relative_kf_pose_enable after YAML/declare.
 * Non-empty GSLAM_INIT_RELATIVE_KF_POSE overrides (1/t/y → true; else false).
 */
inline bool resolve_init_relative_kf_pose_enable(
  bool from_param, const char * env_value)
{
  if (env_value != nullptr && env_value[0] != '\0') {
    const char c = env_value[0];
    return c == '1' || c == 't' || c == 'T' || c == 'y' || c == 'Y';
  }
  return from_param;
}

/**
 * Resolve Init_hybrid_kf_z_absolute_enable after YAML/declare.
 * Non-empty GSLAM_INIT_HYBRID_KF_Z_ABS overrides (1/t/y → true; else false).
 */
inline bool resolve_init_hybrid_kf_z_absolute_enable(
  bool from_param, const char * env_value)
{
  if (env_value != nullptr && env_value[0] != '\0') {
    const char c = env_value[0];
    return c == '1' || c == 't' || c == 'T' || c == 'y' || c == 'Y';
  }
  return from_param;
}

enum class InitMode
{
  Absolute = 0,
  FullRelative = 1,
  HybridXyRelZAbs = 2
};

/**
 * Admission-mode selection used by GMAP::Update and unit tests.
 * First KF / missing EKF history → Absolute regardless of hybrid.
 * Relative OFF → Absolute (hybrid ignored).
 */
inline InitMode select_init_mode(
  bool relative_enable,
  bool hybrid_z_abs_enable,
  bool have_prev_ekf,
  int idx_new)
{
  if (!relative_enable || !have_prev_ekf || idx_new <= 0) {
    return InitMode::Absolute;
  }
  if (hybrid_z_abs_enable) {
    return InitMode::HybridXyRelZAbs;
  }
  return InitMode::FullRelative;
}

inline const char * init_mode_name(InitMode m)
{
  switch (m) {
    case InitMode::Absolute:
      return "absolute";
    case InitMode::FullRelative:
      return "full_relative";
    case InitMode::HybridXyRelZAbs:
      return "hybrid_xy_rel_z_abs";
  }
  return "unknown";
}

struct CamPose
{
  arma::mat::fixed<3, 3> Rc2n;  // = Rn2c^T
  arma::vec::fixed<3> t_c2n;
};

inline CamPose from_keyframe(const KEYFRAME & kf)
{
  CamPose T;
  T.Rc2n = kf.Rn2c.t();
  T.t_c2n = kf.t_c2n;
  return T;
}

inline CamPose invert(const CamPose & T)
{
  CamPose I;
  I.Rc2n = T.Rc2n.t();
  I.t_c2n = -I.Rc2n * T.t_c2n;
  return I;
}

inline CamPose compose(const CamPose & A, const CamPose & B)
{
  CamPose C;
  C.Rc2n = A.Rc2n * B.Rc2n;
  C.t_c2n = A.Rc2n * B.t_c2n + A.t_c2n;
  return C;
}

/** T_GS_i = T_GS_prev * inv(T_EKF_prev) * T_EKF_curr */
inline CamPose propagate(
  const CamPose & T_gs_prev,
  const CamPose & T_ekf_prev,
  const CamPose & T_ekf_curr)
{
  return compose(T_gs_prev, compose(invert(T_ekf_prev), T_ekf_curr));
}

/**
 * KF-Z-A2: relative SE(3) for rotation + XY; Z from absolute EKF camera centre.
 *
 *   T_tmp = propagate(...)
 *   T.R = T_tmp.R
 *   T.t.xy = T_tmp.t.xy
 *   T.t.z = T_ekf_curr.t.z
 *
 * z_offset is not applied (A1: shared nav frame at admission).
 */
inline CamPose propagate_xy_rel_z_abs(
  const CamPose & T_gs_prev,
  const CamPose & T_ekf_prev,
  const CamPose & T_ekf_curr)
{
  CamPose T = propagate(T_gs_prev, T_ekf_prev, T_ekf_curr);
  T.t_c2n(2) = T_ekf_curr.t_c2n(2);
  return T;
}

/**
 * Write camera pose into KEYFRAME and re-derive body fields from extrinsics:
 *   Rn2c = Rc2n^T
 *   Rn2r = Rr2c^T * Rn2c
 *   r_N  = t_c2n - Rn2r^T * t_c2r
 */
inline void apply_cam_pose(KEYFRAME & kf, const CamPose & T)
{
  kf.Rn2c = T.Rc2n.t();
  kf.t_c2n = T.t_c2n;
  kf.Rn2r = kf.Rr2c.t() * kf.Rn2c;
  kf.r_N = kf.t_c2n - kf.Rn2r.t() * kf.t_c2r;
}

/**
 * Opt-in admission Z diagnostic (GSLAM_KF_Z_ADMIT_DIAG=1).
 * Default: silent. Does not alter pose computation.
 */
inline void maybe_log_admit_z(
  InitMode mode,
  int idx_kf,
  uint64_t keyframe_id,
  double z_ekf,
  double z_full_rel,
  double z_admitted,
  double z_gs_prev)
{
  const char * e = std::getenv("GSLAM_KF_Z_ADMIT_DIAG");
  if (e == nullptr || e[0] == '\0') {
    return;
  }
  const char c = e[0];
  if (!(c == '1' || c == 't' || c == 'T' || c == 'y' || c == 'Y')) {
    return;
  }
  std::fprintf(
    stderr,
    "gmap-> kf_z_admit mode=%s idx=%d id=%llu z_ekf=%.6f z_full_rel=%.6f "
    "z_admit=%.6f z_gs_prev=%.6f dz_admit_ekf=%.6f\n",
    init_mode_name(mode),
    idx_kf,
    static_cast<unsigned long long>(keyframe_id),
    z_ekf,
    z_full_rel,
    z_admitted,
    z_gs_prev,
    z_admitted - z_ekf);
}

}  // namespace gmap_relative_kf_init

#endif
