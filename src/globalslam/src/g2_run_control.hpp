#ifndef GLOBALSLAM_G2_RUN_CONTROL_HPP_
#define GLOBALSLAM_G2_RUN_CONTROL_HPP_

#include <cstdint>

#include "g6d_source_policy.hpp"

namespace globalslam
{

/**
 * G2/G6d processing / correction / reset policy (header-only, unit-tested).
 *
 * Semantics:
 * - processing starts disabled (standalone inspection).
 * - Incoming keyframes while disabled are dropped (not queued).
 * - G6d: per-source Enable_* vs Apply_*_*_corrections (see g6d_source_policy).
 * - Apply_global_corrections remains a compatibility master that forces
 *   apply_* for enabled sources only (never silently enables visual processing).
 * - Dispatch boundary: decide under the lock that guards gates, then send
 *   outside without re-checking.
 */
struct G2RunControl
{
  g6d::SourcePolicy sources;
  uint64_t reset_generation{0};
  bool reinit_gmap{false};
  bool reinit_cloop{false};

  void set_processing_enabled(bool on) { sources.processing_enabled = on; }

  bool should_accept_keyframe() const { return sources.should_accept_keyframe(); }

  bool should_emit_local_slam_correction() const
  {
    return sources.should_emit_ba_correction() ||
      sources.should_emit_tag_correction() ||
      sources.should_emit_visual_correction();
  }

  bool should_emit_control_loop_signal() const
  {
    return sources.should_emit_control_loop_signal(g6d::CorrectionSource::TagLoop) ||
      sources.should_emit_control_loop_signal(g6d::CorrectionSource::VisualLoop);
  }

  bool should_emit_for(g6d::CorrectionSource s) const
  {
    return sources.should_emit_for(s);
  }

  bool should_emit_control_for(g6d::CorrectionSource s) const
  {
    return sources.should_emit_control_loop_signal(s);
  }

  bool results_belong_to_generation(uint64_t step_generation) const
  {
    return step_generation == reset_generation;
  }

  void request_reset()
  {
    ++reset_generation;
    reinit_gmap = true;
    reinit_cloop = true;
  }

  bool consume_reinit_gmap()
  {
    if (!reinit_gmap) {
      return false;
    }
    reinit_gmap = false;
    return true;
  }

  bool consume_reinit_cloop()
  {
    if (!reinit_cloop) {
      return false;
    }
    reinit_cloop = false;
    return true;
  }
};

}  // namespace globalslam

#endif  // GLOBALSLAM_G2_RUN_CONTROL_HPP_
