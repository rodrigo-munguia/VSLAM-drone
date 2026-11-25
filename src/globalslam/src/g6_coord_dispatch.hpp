#ifndef GLOBALSLAM_G6_COORD_DISPATCH_HPP_
#define GLOBALSLAM_G6_COORD_DISPATCH_HPP_

/**
 * G6a / LC-CONSISTENCY-A4.1: Local SLAM correction dispatch vs control.
 *
 * A4.1: visual/BA Local feedback is an **estimator-only** transaction. Production
 * `Send_local_slam_pos_update` uses `decide_estimator_only_dispatch` and never
 * calls CoordPause / quiescence / release / resume / legacy SimpleServ 'c'.
 *
 * Coordinate_mode remains for documentation and unit tests of the historical
 * G6a pause protocol (manual/unrelated control coordination is unchanged):
 *   required       — must have pause ack; no control → skip
 *   absent_ok      — if control service missing, dispatch without pause (lab)
 *   offline_bypass — never call control; dispatch directly
 *
 * Control pose for Webots experiments uses Slam_feedback=false (GT/GPS), so
 * discontinuous Local odometry does not drive the flight plan.
 */

#include <cstdint>
#include <string>

namespace globalslam
{
namespace g6_coord
{

enum class CoordMode : uint8_t
{
  Required = 0,
  AbsentOk = 1,
  OfflineBypass = 2,
};

inline CoordMode parse_coord_mode(const std::string & s)
{
  if (s == "absent_ok") {
    return CoordMode::AbsentOk;
  }
  if (s == "offline_bypass") {
    return CoordMode::OfflineBypass;
  }
  return CoordMode::Required;
}

inline const char * coord_mode_name(CoordMode m)
{
  switch (m) {
    case CoordMode::Required: return "required";
    case CoordMode::AbsentOk: return "absent_ok";
    case CoordMode::OfflineBypass: return "offline_bypass";
  }
  return "required";
}

struct DispatchDecision
{
  bool may_dispatch{false};
  bool need_pause{false};
  bool skip_because_no_control{false};
  std::string reason;
};

/**
 * @param apply_corrections Apply_global_corrections
 * @param control_service_ready CorrectionCoordinate service available
 * @param pause_acknowledged prepare_pause succeeded for this transaction
 */
inline DispatchDecision decide_dispatch(
  bool apply_corrections,
  CoordMode mode,
  bool control_service_ready,
  bool pause_acknowledged)
{
  DispatchDecision d;
  if (!apply_corrections) {
    d.reason = "corrections_disabled";
    return d;
  }
  if (mode == CoordMode::OfflineBypass) {
    d.may_dispatch = true;
    d.need_pause = false;
    d.reason = "offline_bypass";
    return d;
  }
  if (!control_service_ready) {
    if (mode == CoordMode::AbsentOk) {
      d.may_dispatch = true;
      d.need_pause = false;
      d.skip_because_no_control = true;
      d.reason = "absent_ok_no_control";
      return d;
    }
    d.skip_because_no_control = true;
    d.reason = "control_required_missing";
    return d;
  }
  d.need_pause = true;
  if (!pause_acknowledged) {
    d.reason = "pause_not_acknowledged";
    return d;
  }
  d.may_dispatch = true;
  d.reason = "pause_acked";
  return d;
}

/** After apply: send legacy SimpleServ 'c' only when NOT using coordinated signal_close_loop. */
inline bool use_legacy_control_c(
  bool coordinated_release_will_signal_cl,
  bool close_loop_type,
  bool applied)
{
  return applied && close_loop_type && !coordinated_release_will_signal_cl;
}

/**
 * A4.1 production dispatch for Local corrections: never request control pause.
 * Equivalent to offline_bypass with corrections enabled.
 */
inline DispatchDecision decide_estimator_only_dispatch(bool apply_corrections)
{
  return decide_dispatch(
    apply_corrections, CoordMode::OfflineBypass,
    /*control_service_ready=*/false, /*pause_acknowledged=*/false);
}

/** True when a dispatch decision requires CoordPause / quiescence. */
inline bool requires_control_pause(const DispatchDecision & d)
{
  return d.need_pause;
}

}  // namespace g6_coord
}  // namespace globalslam

#endif  // GLOBALSLAM_G6_COORD_DISPATCH_HPP_
