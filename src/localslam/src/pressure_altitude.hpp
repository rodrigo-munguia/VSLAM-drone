#ifndef LOCALSLAM_PRESSURE_ALTITUDE_HPP_
#define LOCALSLAM_PRESSURE_ALTITUDE_HPP_

/**
 * A9: FluidPressure → relative altitude (metres, +up) at the ROS boundary.
 *
 * Conversion model (troposphere only)
 * ----------------------------------
 * Primary sources:
 *   - ICAO Doc 7488 / ICAO Standard Atmosphere (ISA) troposphere layer
 *   - Equivalent US Standard Atmosphere 1976 lower atmosphere
 *   - NCAR pressure-altitude notes (same constants / closed form)
 *
 * Absolute ISA geopotential height from absolute static pressure P [Pa]:
 *   h_isa(P) = (T0 / L) * ( 1 - (P / P0)^(R_spec * L / g) )
 *
 * Constants (exact ISA definitions used here):
 *   P0      = 101325 Pa          sea-level standard pressure
 *   T0      = 288.15 K           sea-level standard temperature
 *   L       = 0.0065 K/m         tropospheric temperature lapse magnitude
 *   g       = 9.80665 m/s²       standard gravity
 *   R_spec  = 287.05287 J/(kg·K) specific gas constant for dry air
 *
 * Assumptions / supported conditions:
 *   - Dry ideal gas, hydrostatic equilibrium, constant lapse rate
 *   - Gravity and R_spec independent of altitude
 *   - Absolute ISA height in approximately [-500, 11000] m (troposphere)
 *   - No live temperature, humidity, weather, or bias estimation
 *   - Optional parameter override of T0 only (still ISA closed form)
 *
 * Relative altitude (this slice)
 * ------------------------------
 *   Latch reference pressure P_ref on the first valid sample after
 *   converter reset (policy: first_valid).
 *   h_rel = h_isa(P) - h_isa(P_ref)   [m, +up]
 *
 * Double-offset avoidance vs EKF::Altitude_update:
 *   The EKF still latches the first *internal* altitude sample as home.
 *   Feeding h_rel that is ~0 on the reference sample makes that EKF latch
 *   a no-op (home ≈ 0). Do NOT feed absolute ISA height and also subtract
 *   a ROS home — that would double-offset.
 *
 * Noise:
 *   FluidPressure.variance (Pa²) is IGNORED. Configured Sigma_alt_update
 *   [m] remains the sole altitude measurement noise for the EKF update.
 *
 * Frame / mounting:
 *   Treat fluid_pressure as measured at the same altitude reference point
 *   already assumed by mavros Altitude.relative / Altitude_update (body
 *   altitude state). Optional Pressure_z_lever_arm_m (+up from body origin
 *   to sensor) is subtracted from h_rel so the observation is body-origin
 *   height. Default 0 ⇒ document that sensor height == body-origin height.
 */

#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

namespace localslam
{
namespace pressure_altitude
{

/** ICAO / US Std Atm troposphere constants. */
constexpr double kP0_Pa = 101325.0;
constexpr double kT0_K = 288.15;
constexpr double kLapse_K_per_m = 0.0065;
constexpr double kG_m_s2 = 9.80665;
constexpr double kRspec_J_per_kgK = 287.05287;

/** Default absolute-pressure acceptance band (≈ −500 m … 11 km ISA). */
constexpr double kDefaultPmin_Pa = 22632.0;   // ~11 km
constexpr double kDefaultPmax_Pa = 108900.0;  // ~−500 m

inline double exponent_rl_over_g()
{
  return (kRspec_J_per_kgK * kLapse_K_per_m) / kG_m_s2;
}

/**
 * Absolute ISA geopotential height [m] from absolute pressure [Pa].
 * t0_K is the sea-level temperature used in the closed form (default T0).
 * Returns NaN if inputs are invalid / out of troposphere support.
 */
inline double isa_height_m_from_pressure_pa(double pressure_pa, double t0_K = kT0_K)
{
  if (!std::isfinite(pressure_pa) || !std::isfinite(t0_K) || pressure_pa <= 0.0 ||
    t0_K <= 0.0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double ratio = pressure_pa / kP0_Pa;
  if (!(ratio > 0.0) || !std::isfinite(ratio)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double h =
    (t0_K / kLapse_K_per_m) * (1.0 - std::pow(ratio, exponent_rl_over_g()));
  if (!std::isfinite(h)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return h;
}

/**
 * Inverse: absolute pressure [Pa] from absolute ISA height [m].
 * Independent closed form for tests (not used by the online path).
 */
inline double isa_pressure_pa_from_height_m(double height_m, double t0_K = kT0_K)
{
  if (!std::isfinite(height_m) || !std::isfinite(t0_K) || t0_K <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double inner = 1.0 - (kLapse_K_per_m * height_m) / t0_K;
  if (!(inner > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double exp = kG_m_s2 / (kRspec_J_per_kgK * kLapse_K_per_m);
  const double p = kP0_Pa * std::pow(inner, exp);
  if (!std::isfinite(p) || p <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return p;
}

struct ConvertConfig
{
  double t0_K{kT0_K};
  double p_min_Pa{kDefaultPmin_Pa};
  double p_max_Pa{kDefaultPmax_Pa};
  /** +up lever arm from body altitude origin to pressure sensor [m]. */
  double z_lever_arm_m{0.0};
  std::string expected_frame_id;  // empty = do not enforce
};

/** Relative altitude observation produced at the ROS boundary (+up, metres). */
struct RelativeAltObs
{
  double altitude{0.0};
  long int time{0};
};

inline long int stamp_to_ms(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<long int>(stamp.sec) * 1000L +
         static_cast<long int>(stamp.nanosec) / 1000000L;
}

struct ConvertResult
{
  bool ok{false};
  std::string error;
  RelativeAltObs alt;
  double pressure_pa{0.0};
  double h_isa_m{0.0};
  double h_rel_m{0.0};
  bool latched_new_reference{false};
};

/**
 * Stateful converter: latches P_ref on first valid sample after reset.
 * Thread-safe.
 */
class PressureToRelativeAltitude
{
public:
  void configure(const ConvertConfig & cfg)
  {
    std::lock_guard<std::mutex> lock(mu_);
    cfg_ = cfg;
  }

  /** Clear reference and bump generation (call on ~/reset). */
  void reset_reference()
  {
    std::lock_guard<std::mutex> lock(mu_);
    has_ref_ = false;
    p_ref_Pa_ = 0.0;
    ++generation_;
  }

  uint64_t generation() const
  {
    std::lock_guard<std::mutex> lock(mu_);
    return generation_;
  }

  bool has_reference() const
  {
    std::lock_guard<std::mutex> lock(mu_);
    return has_ref_;
  }

  double reference_pressure_pa() const
  {
    std::lock_guard<std::mutex> lock(mu_);
    return p_ref_Pa_;
  }

  ConvertResult convert(const sensor_msgs::msg::FluidPressure & msg) const
  {
    ConvertResult out;
    ConvertConfig cfg;
    bool has_ref = false;
    double p_ref = 0.0;
    {
      std::lock_guard<std::mutex> lock(mu_);
      cfg = cfg_;
      has_ref = has_ref_;
      p_ref = p_ref_Pa_;
    }

    if (!cfg.expected_frame_id.empty() &&
      msg.header.frame_id != cfg.expected_frame_id)
    {
      out.error =
        "FluidPressure frame_id='" + msg.header.frame_id +
        "' != expected '" + cfg.expected_frame_id + "'";
      return out;
    }

    const double p = msg.fluid_pressure;
    // variance is intentionally ignored (Pa² ≠ altitude m² noise).
    if (!std::isfinite(p)) {
      out.error = "non-finite FluidPressure.fluid_pressure";
      return out;
    }
    if (p < cfg.p_min_Pa || p > cfg.p_max_Pa) {
      out.error = "FluidPressure outside configured [Pmin,Pmax] Pa band";
      return out;
    }

    const double h = isa_height_m_from_pressure_pa(p, cfg.t0_K);
    if (!std::isfinite(h)) {
      out.error = "ISA height conversion failed";
      return out;
    }

    out.pressure_pa = p;
    out.h_isa_m = h;

    double h_rel = 0.0;
    bool latched = false;
    {
      std::lock_guard<std::mutex> lock(mu_);
      if (!has_ref_) {
        has_ref_ = true;
        p_ref_Pa_ = p;
        has_ref = true;
        p_ref = p;
        latched = true;
      } else {
        has_ref = has_ref_;
        p_ref = p_ref_Pa_;
      }
    }

    const double h_ref = isa_height_m_from_pressure_pa(p_ref, cfg.t0_K);
    if (!std::isfinite(h_ref)) {
      out.error = "ISA height of reference pressure failed";
      return out;
    }
    h_rel = h - h_ref;
    // Move observation to body-origin altitude if a lever arm is configured.
    h_rel -= cfg.z_lever_arm_m;

    out.h_rel_m = h_rel;
    out.latched_new_reference = latched;
    out.alt.altitude = h_rel;
    out.alt.time = stamp_to_ms(msg.header.stamp);
    out.ok = true;
    (void)has_ref;
    return out;
  }

private:
  mutable std::mutex mu_;
  ConvertConfig cfg_;
  mutable bool has_ref_{false};
  mutable double p_ref_Pa_{0.0};
  uint64_t generation_{0};
};

}  // namespace pressure_altitude
}  // namespace localslam

#endif  // LOCALSLAM_PRESSURE_ALTITUDE_HPP_
