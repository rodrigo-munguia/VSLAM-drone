/**
 * G6c→G6d regression: independent BA vs tag feedback gates.
 * Keeps match-index / coord / legacy-c contracts from the G6c audit.
 */

#include <gtest/gtest.h>

#include "g2_run_control.hpp"
#include "g6b_cl_match_index.hpp"
#include "g6_coord_dispatch.hpp"
#include "g6d_source_policy.hpp"

using globalslam::G2RunControl;
using globalslam::g6b_cl::resolve_matched_kf_index;
using globalslam::g6_coord::decide_dispatch;
using globalslam::g6_coord::parse_coord_mode;
using globalslam::g6_coord::use_legacy_control_c;
using globalslam::g6d::CorrectionSource;

TEST(G6cOptFeedbackAudit, IndependentBaVsTagFeedback)
{
  G2RunControl c;
  c.set_processing_enabled(true);
  c.sources.enable_global_ba = true;
  c.sources.apply_ba_corrections = false;
  c.sources.enable_tag_loop_closure = true;
  c.sources.apply_tag_loop_corrections = true;
  c.sources.enable_visual_loop_closure = false;
  EXPECT_FALSE(c.should_emit_for(CorrectionSource::Ba));
  EXPECT_TRUE(c.should_emit_for(CorrectionSource::TagLoop));
  EXPECT_TRUE(c.should_emit_control_loop_signal());
  EXPECT_TRUE(c.should_emit_control_for(CorrectionSource::TagLoop));
}

TEST(G6cOptFeedbackAudit, MasterApplyOpensEnabledSourcesOnly)
{
  G2RunControl c;
  c.set_processing_enabled(true);
  c.sources.enable_global_ba = true;
  c.sources.enable_tag_loop_closure = true;
  c.sources.enable_visual_loop_closure = false;
  c.sources.apply_global_corrections_master = true;
  c.sources.apply_master_compatibility();
  EXPECT_TRUE(c.should_emit_for(CorrectionSource::Ba));
  EXPECT_TRUE(c.should_emit_for(CorrectionSource::TagLoop));
  EXPECT_FALSE(c.should_emit_for(CorrectionSource::VisualLoop));
}

TEST(G6cOptFeedbackAudit, ProcessingAloneDoesNotEmitFeedback)
{
  G2RunControl c;
  c.set_processing_enabled(true);
  EXPECT_TRUE(c.should_accept_keyframe());
  EXPECT_FALSE(c.should_emit_local_slam_correction());
}

TEST(G6cOptFeedbackAudit, TagOnlyLoopSkipsOptionalMatchBaNotPoseSlamCommitPolicy)
{
  const auto d = resolve_matched_kf_index(-1, /*ref=*/0, /*n=*/280);
  EXPECT_FALSE(d.valid);
  EXPECT_STREQ(d.reason, "no_corresponding_visual_match");
}

TEST(G6cOptFeedbackAudit, VisualMatchAllowsOptionalRefinement)
{
  const auto d = resolve_matched_kf_index(12, 0, 280);
  EXPECT_TRUE(d.valid);
  EXPECT_STREQ(d.reason, "visual_match");
}

TEST(G6cOptFeedbackAudit, CorrectionsOffDropsDispatchEntirely)
{
  const auto mode = parse_coord_mode("required");
  const auto d = decide_dispatch(
    /*apply=*/false, mode, /*control_ready=*/true, /*pause_acked=*/false);
  EXPECT_FALSE(d.may_dispatch);
  EXPECT_FALSE(d.need_pause);
}

TEST(G6cOptFeedbackAudit, LegacyControlCRequiresAppliedCloseLoop)
{
  EXPECT_TRUE(use_legacy_control_c(false, true, true));
  EXPECT_FALSE(use_legacy_control_c(false, true, false));
  EXPECT_FALSE(use_legacy_control_c(false, false, true));
  EXPECT_FALSE(use_legacy_control_c(true, true, true));
}

TEST(G6cOptFeedbackAudit, SlamFeedbackIsNotApplyCorrections)
{
  const bool slam_feedback = true;
  const bool tag_apply = false;
  EXPECT_TRUE(slam_feedback);
  EXPECT_FALSE(tag_apply);
  G2RunControl c;
  c.set_processing_enabled(true);
  c.sources.apply_tag_loop_corrections = tag_apply;
  EXPECT_FALSE(c.should_emit_control_loop_signal());
}
