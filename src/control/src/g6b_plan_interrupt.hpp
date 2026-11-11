#ifndef CONTROL_G6B_PLAN_INTERRUPT_HPP_
#define CONTROL_G6B_PLAN_INTERRUPT_HPP_

#include <cstdint>

/**
 * G6b.1: classify why go_point / exploration left its run loop.
 *
 * prepare_pause clears run_control_plan so in-flight commands can become
 * quiescent. That must not be treated as command failure or plan advance:
 * while coord_suppress is armed, the command parks and re-enters the same
 * target after resume.
 */

namespace control
{
namespace g6b_plan
{

enum class InterruptKind : uint8_t
{
  None = 0,
  /** Temporary coordination pause — park, do not fail the command. */
  CoordPause = 1,
  /** Operator stop / plan disabled without active suppress. */
  OperatorStop = 2,
};

/**
 * @param run_control_plan current plan enable bit
 * @param coord_suppress   CorrectionCoordinate suppress armed
 */
inline InterruptKind classify_run_clear(bool run_control_plan, bool coord_suppress)
{
  if (run_control_plan) {
    return InterruptKind::None;
  }
  if (coord_suppress) {
    return InterruptKind::CoordPause;
  }
  return InterruptKind::OperatorStop;
}

/** After a park wait: should the navigation loop continue toward the same target? */
inline bool should_resume_same_target(bool run_control_plan, bool coord_suppress)
{
  return run_control_plan && !coord_suppress;
}

}  // namespace g6b_plan
}  // namespace control

#endif  // CONTROL_G6B_PLAN_INTERRUPT_HPP_
