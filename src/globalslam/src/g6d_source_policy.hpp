#ifndef GLOBALSLAM_G6D_SOURCE_POLICY_HPP_
#define GLOBALSLAM_G6D_SOURCE_POLICY_HPP_

/**
 * G6d: independent Global processing vs Local-feedback gates per source.
 *
 * Sources: ba | tag_loop | visual_loop
 *
 * Feedback requires its source processing to be enabled.
 * Tag + visual processing together is rejected in this slice (no racing /
 * arbitration). Visual processing uses existing CLOOP::Step (ORB/PnP); default
 * off until exercised. Post-loop BA refine shares Enable_global_ba and does
 * not emit a second BA Local-feedback request.
 */

#include <cstdint>
#include <string>

namespace globalslam
{
namespace g6d
{

enum class CorrectionSource : uint8_t
{
  Ba = 0,
  TagLoop = 1,
  VisualLoop = 2,
};

inline const char * source_name(CorrectionSource s)
{
  switch (s) {
    case CorrectionSource::Ba: return "ba";
    case CorrectionSource::TagLoop: return "tag_loop";
    case CorrectionSource::VisualLoop: return "visual_loop";
  }
  return "unknown";
}

/** Map to LSposUpdate.type for existing GS/CL apply gates. */
inline const char * ls_update_type_for(CorrectionSource s)
{
  return (s == CorrectionSource::Ba) ? "bundle_adjustment" : "close_loop";
}

struct SourcePolicy
{
  bool processing_enabled{false};

  bool enable_global_ba{true};
  bool enable_tag_loop_closure{true};
  bool enable_visual_loop_closure{false};

  bool apply_ba_corrections{false};
  bool apply_tag_loop_corrections{false};
  bool apply_visual_loop_corrections{false};

  /**
   * Compatibility: when true at configure time, force apply_* = true for each
   * enabled source (not visual unless visual processing is also enabled).
   * Does not enable visual processing by itself.
   */
  bool apply_global_corrections_master{false};

  uint64_t map_revision{0};

  void apply_master_compatibility()
  {
    if (!apply_global_corrections_master) {
      return;
    }
    if (enable_global_ba) {
      apply_ba_corrections = true;
    }
    if (enable_tag_loop_closure) {
      apply_tag_loop_corrections = true;
    }
    if (enable_visual_loop_closure) {
      apply_visual_loop_corrections = true;
    }
  }

  /** Validate; on failure set reason and return false. */
  bool validate(std::string & reason) const
  {
    if (enable_tag_loop_closure && enable_visual_loop_closure) {
      reason =
        "Enable_tag_loop_closure and Enable_visual_loop_closure cannot both "
        "be true in this slice (no simultaneous loop-source arbitration). "
        "Pick one processing path.";
      return false;
    }
    if (apply_ba_corrections && !enable_global_ba) {
      reason = "Apply_ba_corrections requires Enable_global_ba";
      return false;
    }
    if (apply_tag_loop_corrections && !enable_tag_loop_closure) {
      reason = "Apply_tag_loop_corrections requires Enable_tag_loop_closure";
      return false;
    }
    if (apply_visual_loop_corrections && !enable_visual_loop_closure) {
      reason =
        "Apply_visual_loop_corrections requires Enable_visual_loop_closure";
      return false;
    }
    reason.clear();
    return true;
  }

  bool should_accept_keyframe() const { return processing_enabled; }

  bool should_run_ba() const
  {
    return processing_enabled && enable_global_ba;
  }

  bool should_run_tag_loop() const
  {
    return processing_enabled && enable_tag_loop_closure;
  }

  bool should_run_visual_loop() const
  {
    return processing_enabled && enable_visual_loop_closure;
  }

  /** Post-loop BA refine shares Enable_global_ba (not a second feedback source). */
  bool should_run_loop_ba_refine() const { return should_run_ba(); }

  bool should_emit_ba_correction() const
  {
    return should_run_ba() && apply_ba_corrections;
  }

  bool should_emit_tag_correction() const
  {
    return should_run_tag_loop() && apply_tag_loop_corrections;
  }

  bool should_emit_visual_correction() const
  {
    return should_run_visual_loop() && apply_visual_loop_corrections;
  }

  bool should_emit_for(CorrectionSource s) const
  {
    switch (s) {
      case CorrectionSource::Ba: return should_emit_ba_correction();
      case CorrectionSource::TagLoop: return should_emit_tag_correction();
      case CorrectionSource::VisualLoop: return should_emit_visual_correction();
    }
    return false;
  }

  /** Mission re-nav signal only when the corresponding loop feedback applies. */
  bool should_emit_control_loop_signal(CorrectionSource s) const
  {
    return should_emit_for(s) &&
      (s == CorrectionSource::TagLoop || s == CorrectionSource::VisualLoop);
  }

  std::string matrix_summary() const
  {
    return std::string("proc=") + (processing_enabled ? "1" : "0") +
      " ba=" + (enable_global_ba ? "1" : "0") +
      "/fb=" + (apply_ba_corrections ? "1" : "0") +
      " tag=" + (enable_tag_loop_closure ? "1" : "0") +
      "/fb=" + (apply_tag_loop_corrections ? "1" : "0") +
      " vis=" + (enable_visual_loop_closure ? "1" : "0") +
      "/fb=" + (apply_visual_loop_corrections ? "1" : "0") +
      " master_apply=" + (apply_global_corrections_master ? "1" : "0");
  }
};

}  // namespace g6d
}  // namespace globalslam

#endif  // GLOBALSLAM_G6D_SOURCE_POLICY_HPP_
