#ifndef LOCALSLAM_RANGE_FRAME_ASSOCIATION_HPP_
#define LOCALSLAM_RANGE_FRAME_ASSOCIATION_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

namespace localslam
{

/**
 * Downward-range ↔ image association used inside getData().
 *
 * Times are milliseconds (legacy DATA.time units).
 *
 * A1 behavior change vs pre-A1 Webots runs:
 * - Legacy bridge left custom message times at 0, so
 *   dt = frame.time - last_range_time was typically 0, and the
 *   (dt > kAltFallbackDtMs) altitude-as-depth fallback rarely/never fired.
 * - A1 stamps from headers (or node clock for headerless sources), so dt is
 *   meaningful and the fallback can activate. This is NOT behavior-preserving.
 *
 * Prediction scheduling is unrelated and remains wall-clock.
 *
 * B2: association / undelayed-init limits are ROS parameters
 * (Range_assoc_min_m, Range_assoc_max_m, Range_init_min_m) with sonar
 * defaults matching the former hardcoded gates below.
 *
 * B4: Range_allow_altitude_fallback (default true) gates only the
 * altitude→FRAME.range path. Altitude EKF updates are independent.
 */
struct RangeFrameAssociationState
{
  double last_range{0.0};
  long int last_range_time{0};
  bool range_available{false};
  bool range_available_alt{false};
  double last_alt{0.0};
};

constexpr long int kAltFallbackDtMs = 100;
/** Sonar-equivalent exclusive association lower bound (metres). */
constexpr double kRangeGateMinM = 0.4;
/** Sonar-equivalent exclusive association upper bound (metres). */
constexpr double kRangeGateMaxM = 6.0;
/** Sonar-equivalent exclusive undelayed_range init lower bound (metres). */
constexpr double kRangeInitMinM = 0.5;

/**
 * Validate configurable range acceptance / init thresholds.
 * Throws std::runtime_error on invalid configuration (no silent defaults).
 */
inline void validate_range_acceptance_params(
  double assoc_min_m, double assoc_max_m, double init_min_m)
{
  if (!std::isfinite(assoc_min_m) || !std::isfinite(assoc_max_m) ||
    !std::isfinite(init_min_m))
  {
    throw std::runtime_error(
      "Range_assoc_min_m / Range_assoc_max_m / Range_init_min_m must be finite");
  }
  if (assoc_min_m < 0.0) {
    throw std::runtime_error(
      "Range_assoc_min_m must be >= 0 (got " + std::to_string(assoc_min_m) + ")");
  }
  if (!(assoc_max_m > assoc_min_m)) {
    throw std::runtime_error(
      "Range_assoc_max_m must be > Range_assoc_min_m (got max=" +
      std::to_string(assoc_max_m) + " min=" + std::to_string(assoc_min_m) + ")");
  }
  if (init_min_m < 0.0) {
    throw std::runtime_error(
      "Range_init_min_m must be >= 0 (got " + std::to_string(init_min_m) + ")");
  }
  if (!(init_min_m < assoc_max_m)) {
    throw std::runtime_error(
      "Range_init_min_m must be < Range_assoc_max_m (got init=" +
      std::to_string(init_min_m) + " max=" + std::to_string(assoc_max_m) + ")");
  }
}

/** Exclusive association gate: gate_min_m < range_m < gate_max_m. */
inline bool range_passes_assoc_gate(
  double range_m, double gate_min_m, double gate_max_m)
{
  return range_m > gate_min_m && range_m < gate_max_m;
}

/** Exclusive undelayed_range init gate: range_m > init_min_m. */
inline bool range_permits_undelayed_init(double range_m, double init_min_m)
{
  return range_m > init_min_m;
}

/**
 * Record a downward-range sample if it passes the configured association gate.
 * Defaults preserve legacy sonar exclusive bounds (0.4, 6.0).
 */
inline bool note_downward_range(
  RangeFrameAssociationState & s, double range_m, long int time_ms,
  double gate_min_m = kRangeGateMinM, double gate_max_m = kRangeGateMaxM)
{
  if (range_passes_assoc_gate(range_m, gate_min_m, gate_max_m)) {
    s.last_range = range_m;
    s.last_range_time = time_ms;
    s.range_available = true;
    return true;
  }
  return false;
}

inline void note_altitude_for_fallback(
  RangeFrameAssociationState & s, double altitude_m)
{
  s.last_alt = altitude_m;
  s.range_available_alt = true;
}

/**
 * Attach depth hint to a frame.
 * Prefer pending camera-aligned range; else if allow_altitude_fallback and
 * |frame_time - last_range_time| > 100 ms and a positive altitude sample is
 * pending, use altitude as depth hint (legacy sonar default).
 *
 * When allow_altitude_fallback is false, altitude never populates or
 * overwrites FRAME.range, and any pending altitude-depth latch is cleared so
 * it cannot linger for a later frame.
 *
 * Returns the range_type string assigned ("", "sonar", or "alt").
 * ("sonar" is historical — the pending sample may be laser via remap.)
 */
inline std::string associate_depth_to_frame(
  RangeFrameAssociationState & s,
  double & frame_range_inout,
  long int frame_time_ms,
  bool allow_altitude_fallback = true)
{
  std::string range_type;
  if (s.range_available) {
    frame_range_inout = s.last_range;
    range_type = "sonar";
    s.range_available = false;
  }
  const long int dt = frame_time_ms - s.last_range_time;
  if (allow_altitude_fallback && s.range_available_alt &&
    (dt > kAltFallbackDtMs) && (s.last_alt > 0))
  {
    frame_range_inout = s.last_alt;
    range_type = "alt";
    s.range_available_alt = false;
  } else if (!allow_altitude_fallback && s.range_available_alt) {
    // Drop unused altitude-depth latch; EKF altitude updates are unaffected.
    s.range_available_alt = false;
  }
  return range_type;
}

/**
 * B4: Effective confidence-region classification used only for σ_ini selection
 * in undelayed_range init (ekf_visual_init_feats.cpp). Does not affect feature
 * eligibility. Mirrors production predicate:
 *   (dp_px < r_range_image_px) && (range_m < max_range_operation_m)
 */
inline bool feature_in_range_confidence_region(
  double dp_px, double r_range_image_px,
  double range_m, double max_range_operation_m)
{
  return (dp_px < r_range_image_px) && (range_m < max_range_operation_m);
}

/** First-landmark multiplier applied in production (*10 when FeatsDATA empty). */
constexpr double kFirstLandmarkDepthSigmaScale = 10.0;

inline double initial_depth_sigma_m(
  bool in_confidence_region, bool has_existing_feats,
  double sigma_in_region_m, double sigma_wo_region_m)
{
  const double base =
    in_confidence_region ? sigma_in_region_m : sigma_wo_region_m;
  return has_existing_feats ? base : (base * kFirstLandmarkDepthSigmaScale);
}

}  // namespace localslam

#endif  // LOCALSLAM_RANGE_FRAME_ASSOCIATION_HPP_
