#ifndef EKF_JAC_H
#define EKF_JAC_H

#include <cmath>
#include <cstring>
#include <armadillo>
#include "opencv2/opencv.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "../../common/Vision/vision.hpp"




void Jac_quat_noise_driven_model2(arma::vec& x, double delta_t, double Tau_r, double Tau_p, arma::mat::fixed<13,13>& Jfx, arma::mat::fixed<13,6>& Jfu);   
void Jac_euler_noise_driven_model(arma::vec& x, double delta_t, double Tau_r, double Tau_p, arma::mat::fixed<13,13>& Jfx, arma::mat::fixed<13,6>& Jfu);

void Jac_euler_diff_driven_model(arma::vec& x, double delta_t, double R, double L,  arma::mat::fixed<13,13>& Jfx, arma::mat::fixed<13,2>& Jfu);


void Jac_e_init_feat_func(cv::Point2d uvd, arma::vec& x, CAM &camera_parameters, arma::mat &Rc2r,arma::vec::fixed<3> &t_c2r, double depth,arma::mat::fixed<3,13>& dy_dx, arma::mat::fixed<3,3>& dy_duvr);

/** E5e-G2: Jacobians for undelayed_range horizontal ground-plane initialization.
 *  p = p_C + λ m,  λ = r_eff * v_z / m_z.
 *  Measurement channel 2 is Euclidean slant range (same contract as Isaac optical beam).
 *  Inverse projection uses return_Jac=true so dhc_duvd is populated (same as legacy Jac).
 */
void Jac_e_init_feat_wr_horizontal(
  cv::Point2d uvd, arma::vec & x, CAM & camera_parameters, arma::mat & Rc2r,
  arma::vec::fixed<3> & t_c2r, double range_eff,
  arma::mat::fixed<3, 13> & dy_dx, arma::mat::fixed<3, 3> & dy_duvr);

void Jac_euler_2_rotMat_by_P(arma::mat::fixed<3,3> &dfm_de, arma::vec::fixed<3> &p, double phi, double theta, double psi);
void Jac_e_uv_XYZ(arma::vec::fixed<3> pxyz, arma::vec& x, CAM &camera_parameters, arma::mat &Rr2c,arma::vec::fixed<3> &t_c2r,arma::mat::fixed<2,13>& duv_dx, arma::mat::fixed<2,3>& duv_dy,bool Visual_update_attitude_update);

/** Minimum |m_z| for a stable horizontal-plane ray intersection (unitless).
 *  |m_z| < this ⇒ feature ray within ~0.057° of the horizontal plane → reject.
 */
constexpr double kRangeHorizPlaneMinAbsMz = 1.0e-3;

/** λ = r_eff * (e_z·v) / (e_z·m) for horizontal plane through optical-axis hit.
 *  Returns false when range/bearing invalid, ray nearly parallel to the plane, or λ ≤ 0.
 */
inline bool range_horizontal_plane_lambda(
  const arma::vec::fixed<3> & m_nav,
  const arma::vec::fixed<3> & optical_nav,
  double range_eff,
  double * lambda_out)
{
  if (lambda_out == nullptr) {
    return false;
  }
  const double mz = m_nav(2);
  const double vz = optical_nav(2);
  if (!std::isfinite(range_eff) || range_eff <= 0.0) {
    return false;
  }
  if (!std::isfinite(mz) || !std::isfinite(vz)) {
    return false;
  }
  if (std::abs(mz) < kRangeHorizPlaneMinAbsMz) {
    return false;
  }
  const double lambda = range_eff * vz / mz;
  if (!std::isfinite(lambda) || lambda <= 0.0) {
    return false;
  }
  *lambda_out = lambda;
  return true;
}

#endif