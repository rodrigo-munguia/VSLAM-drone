#ifndef LOCALSLAM_LS_POS_APPLY_POLICY_HPP_
#define LOCALSLAM_LS_POS_APPLY_POLICY_HPP_

/**
 * G5: decide which translation axes of an admitted LSposUpdate are applied.
 *
 * Gates (existing YAML semantics, magnitude = absolute value):
 *   bundle_adjustment → GS_xy_update / GS_z_update + max deltas
 *   close_loop        → CL_xy_update / CL_z_update + max deltas
 *
 * XY uses hypot(dx,dy). Z uses |dz| (historical code compared signed dz < max,
 * which accepted arbitrarily large negative Z — treated as a defect in G5).
 *
 * Per-axis: XY pair is all-or-nothing; Z is independent. A request may therefore
 * apply only XY, only Z, both, or neither (gated_noop).
 *
 * Covariance: Update_pos_with_delta does not touch P (documented as-is).
 * close_loop Pc reset occurs only when at least one axis is applied.
 */

#include <cmath>
#include <string>

namespace localslam
{

struct PosUpdateGateParams
{
  bool gs_xy{true};
  bool gs_z{true};
  double gs_xy_max{0.1};
  double gs_z_max{0.1};
  bool cl_xy{true};
  bool cl_z{true};
  double cl_xy_max{5.0};
  double cl_z_max{1.0};
};

struct PosUpdateApplyPlan
{
  bool apply_xy{false};
  bool apply_z{false};
  double applied_dx{0.0};
  double applied_dy{0.0};
  double applied_dz{0.0};
  bool any_applied{false};
  bool reset_pc{false};  // close_loop only, and only if any_applied
};

inline double hypot2(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

inline PosUpdateApplyPlan plan_pos_update_apply(
  double dx, double dy, double dz,
  const std::string & type,
  const PosUpdateGateParams & g)
{
  PosUpdateApplyPlan plan;
  const double innov_xy = hypot2(dx, dy);
  const double mag_z = std::abs(dz);

  if (type == "bundle_adjustment") {
    plan.apply_xy = g.gs_xy && (innov_xy < g.gs_xy_max);
    plan.apply_z = g.gs_z && (mag_z < g.gs_z_max);
  } else if (type == "close_loop") {
    plan.apply_xy = g.cl_xy && (innov_xy < g.cl_xy_max);
    plan.apply_z = g.cl_z && (mag_z < g.cl_z_max);
    plan.reset_pc = false;  // set below if any applied
  } else {
    return plan;
  }

  if (plan.apply_xy) {
    plan.applied_dx = dx;
    plan.applied_dy = dy;
  }
  if (plan.apply_z) {
    plan.applied_dz = dz;
  }
  plan.any_applied = plan.apply_xy || plan.apply_z;
  if (type == "close_loop" && plan.any_applied) {
    plan.reset_pc = true;
  }
  return plan;
}

/** Public odom displacement for an applied internal translation: D=diag(1,-1,-1). */
inline void applied_internal_to_public(
  double dx_i, double dy_i, double dz_i,
  double & dx_p, double & dy_p, double & dz_p)
{
  dx_p = dx_i;
  dy_p = -dy_i;
  dz_p = -dz_i;
}

}  // namespace localslam

#endif  // LOCALSLAM_LS_POS_APPLY_POLICY_HPP_
