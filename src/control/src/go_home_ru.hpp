#ifndef CONTROL_GO_HOME_RU_HPP_
#define CONTROL_GO_HOME_RU_HPP_

/**
 * go_home uncertainty-area branch decision (control_commands.cpp).
 *
 * When r_u is finite (including 0 after Pc reset/init): explore_area with
 * x_a = y_a = r_u — matches the original controller.
 * When r_u is non-finite (never received / cleared on reinit): skip
 * exploration explicitly — do not invent a radius.
 */

#include <cmath>

namespace control
{
namespace go_home_ru
{

enum class ExploreDecision
{
  SkipUnavailable,
  ExploreWithRadius,
};

/**
 * Decide whether go_home should call explore_area, and with what half-extents.
 * Matches ControlQuad::go_home sizing: x_a = y_a = r_u.
 */
inline ExploreDecision decide_explore(double r_u, double & x_a, double & y_a)
{
  if (!std::isfinite(r_u)) {
    x_a = 0.0;
    y_a = 0.0;
    return ExploreDecision::SkipUnavailable;
  }
  x_a = r_u;
  y_a = r_u;
  return ExploreDecision::ExploreWithRadius;
}

/** Same explore sizing for equivalent finite r_u values. */
inline bool explore_sizing_equivalent(double r_u_a, double r_u_b, double tol = 1e-12)
{
  double xa = 0, ya = 0, xb = 0, yb = 0;
  const auto da = decide_explore(r_u_a, xa, ya);
  const auto db = decide_explore(r_u_b, xb, yb);
  if (da != db) {
    return false;
  }
  if (da == ExploreDecision::SkipUnavailable) {
    return true;
  }
  return std::abs(xa - xb) <= tol && std::abs(ya - yb) <= tol;
}

}  // namespace go_home_ru
}  // namespace control

#endif  // CONTROL_GO_HOME_RU_HPP_
