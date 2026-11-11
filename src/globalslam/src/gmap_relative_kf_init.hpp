/**
 * GSLAM-BA-D6: opt-in relative keyframe pose initialization helpers.
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
 * Default-disabled. No BA solver changes.
 */
#ifndef GMAP_RELATIVE_KF_INIT_HPP
#define GMAP_RELATIVE_KF_INIT_HPP

#include <armadillo>

#include "../../common/keyframe_types.hpp"

namespace gmap_relative_kf_init
{

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

}  // namespace gmap_relative_kf_init

#endif
