#ifndef WEBOTBRIDGE_RN2R_INIT_HPP_
#define WEBOTBRIDGE_RN2R_INIT_HPP_

#include <armadillo>

namespace webotbridge
{

/**
 * Initialize nav→robot rotation to 3×3 identity until the first IMU sample.
 *
 * Armadillo pitfall (CTRL-S3b): default-constructed arma::mat is 0×0, and
 * mat::eye() with no size arguments is a no-op on an empty matrix. GPS speed
 * callbacks that compute Rn2rG * v_enu then throw:
 *   matrix multiplication: incompatible matrix dimensions: 0x0 and 3x1
 * Identity until attitude is available is the correct temporary transform.
 */
inline void init_rn2rg_identity(arma::mat & Rn2rG)
{
  Rn2rG.eye(3, 3);
}

inline bool rn2rg_is_3x3(const arma::mat & Rn2rG)
{
  return Rn2rG.n_rows == 3 && Rn2rG.n_cols == 3;
}

}  // namespace webotbridge

#endif  // WEBOTBRIDGE_RN2R_INIT_HPP_
