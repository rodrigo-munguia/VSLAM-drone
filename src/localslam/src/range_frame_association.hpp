#ifndef LOCALSLAM_RANGE_FRAME_ASSOCIATION_HPP_
#define LOCALSLAM_RANGE_FRAME_ASSOCIATION_HPP_

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
constexpr double kRangeGateMinM = 0.4;
constexpr double kRangeGateMaxM = 6.0;

/** Record a downward-range sample if it passes the legacy gate. */
inline bool note_downward_range(
  RangeFrameAssociationState & s, double range_m, long int time_ms)
{
  if (range_m > kRangeGateMinM && range_m < kRangeGateMaxM) {
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
 * Prefer pending sonar range; else if |frame_time - last_range_time| > 100 ms
 * and a positive altitude sample is pending, use altitude as depth hint.
 *
 * Returns the range_type string assigned ("", "sonar", or "alt").
 */
inline std::string associate_depth_to_frame(
  RangeFrameAssociationState & s,
  double & frame_range_inout,
  long int frame_time_ms)
{
  std::string range_type;
  if (s.range_available) {
    frame_range_inout = s.last_range;
    range_type = "sonar";
    s.range_available = false;
  }
  const long int dt = frame_time_ms - s.last_range_time;
  if (s.range_available_alt && (dt > kAltFallbackDtMs) && (s.last_alt > 0)) {
    frame_range_inout = s.last_alt;
    range_type = "alt";
    s.range_available_alt = false;
  }
  return range_type;
}

}  // namespace localslam

#endif  // LOCALSLAM_RANGE_FRAME_ASSOCIATION_HPP_
