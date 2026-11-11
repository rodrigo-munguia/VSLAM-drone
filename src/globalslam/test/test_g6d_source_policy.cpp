#include <gtest/gtest.h>

#include "g6d_source_policy.hpp"
#include "g2_run_control.hpp"

using globalslam::G2RunControl;
using globalslam::g6d::CorrectionSource;
using globalslam::g6d::SourcePolicy;
using globalslam::g6d::ls_update_type_for;
using globalslam::g6d::source_name;

TEST(G6dSourcePolicy, DefaultsKeepFeedbackOff)
{
  SourcePolicy p;
  EXPECT_FALSE(p.processing_enabled);
  EXPECT_TRUE(p.enable_global_ba);
  EXPECT_TRUE(p.enable_tag_loop_closure);
  EXPECT_FALSE(p.enable_visual_loop_closure);
  EXPECT_FALSE(p.apply_ba_corrections);
  EXPECT_FALSE(p.apply_tag_loop_corrections);
  EXPECT_FALSE(p.apply_visual_loop_corrections);
  std::string reason;
  EXPECT_TRUE(p.validate(reason));
}

TEST(G6dSourcePolicy, IndependentBaOffTagFeedbackOn)
{
  SourcePolicy p;
  p.processing_enabled = true;
  p.enable_global_ba = true;
  p.apply_ba_corrections = false;
  p.enable_tag_loop_closure = true;
  p.apply_tag_loop_corrections = true;
  p.enable_visual_loop_closure = false;
  std::string reason;
  ASSERT_TRUE(p.validate(reason)) << reason;
  EXPECT_FALSE(p.should_emit_ba_correction());
  EXPECT_TRUE(p.should_emit_tag_correction());
  EXPECT_TRUE(p.should_emit_control_loop_signal(CorrectionSource::TagLoop));
  EXPECT_FALSE(p.should_emit_control_loop_signal(CorrectionSource::Ba));
}

TEST(G6dSourcePolicy, RejectDualLoopSources)
{
  SourcePolicy p;
  p.enable_tag_loop_closure = true;
  p.enable_visual_loop_closure = true;
  std::string reason;
  EXPECT_FALSE(p.validate(reason));
  EXPECT_NE(reason.find("cannot both"), std::string::npos);
}

TEST(G6dSourcePolicy, FeedbackRequiresProcessing)
{
  SourcePolicy p;
  p.apply_ba_corrections = true;
  p.enable_global_ba = false;
  std::string reason;
  EXPECT_FALSE(p.validate(reason));
  EXPECT_NE(reason.find("Enable_global_ba"), std::string::npos);

  p.enable_global_ba = true;
  p.apply_tag_loop_corrections = true;
  p.enable_tag_loop_closure = false;
  EXPECT_FALSE(p.validate(reason));
}

TEST(G6dSourcePolicy, MasterApplyDoesNotEnableVisualProcessing)
{
  SourcePolicy p;
  p.enable_global_ba = true;
  p.enable_tag_loop_closure = true;
  p.enable_visual_loop_closure = false;
  p.apply_global_corrections_master = true;
  p.apply_master_compatibility();
  EXPECT_TRUE(p.apply_ba_corrections);
  EXPECT_TRUE(p.apply_tag_loop_corrections);
  EXPECT_FALSE(p.apply_visual_loop_corrections);
  EXPECT_FALSE(p.enable_visual_loop_closure);
}

TEST(G6dSourcePolicy, ProcessingDisableSuppressesAllWork)
{
  SourcePolicy p;
  p.processing_enabled = false;
  p.enable_global_ba = true;
  p.apply_ba_corrections = true;
  p.enable_tag_loop_closure = true;
  p.apply_tag_loop_corrections = true;
  EXPECT_FALSE(p.should_run_ba());
  EXPECT_FALSE(p.should_run_tag_loop());
  EXPECT_FALSE(p.should_emit_ba_correction());
  EXPECT_FALSE(p.should_emit_tag_correction());
}

TEST(G6dSourcePolicy, LoopBaRefineSharesEnableGlobalBa)
{
  SourcePolicy p;
  p.processing_enabled = true;
  p.enable_global_ba = false;
  EXPECT_FALSE(p.should_run_loop_ba_refine());
  p.enable_global_ba = true;
  EXPECT_TRUE(p.should_run_loop_ba_refine());
}

TEST(G6dSourcePolicy, LsUpdateTypeMapping)
{
  EXPECT_STREQ(ls_update_type_for(CorrectionSource::Ba), "bundle_adjustment");
  EXPECT_STREQ(ls_update_type_for(CorrectionSource::TagLoop), "close_loop");
  EXPECT_STREQ(ls_update_type_for(CorrectionSource::VisualLoop), "close_loop");
  EXPECT_STREQ(source_name(CorrectionSource::TagLoop), "tag_loop");
}

TEST(G6dSourcePolicy, G2RunControlUsesSources)
{
  G2RunControl c;
  c.sources.processing_enabled = true;
  c.sources.enable_tag_loop_closure = true;
  c.sources.apply_tag_loop_corrections = true;
  c.sources.enable_global_ba = true;
  c.sources.apply_ba_corrections = false;
  EXPECT_TRUE(c.should_emit_local_slam_correction());
  EXPECT_TRUE(c.should_emit_control_loop_signal());
  EXPECT_FALSE(c.should_emit_for(CorrectionSource::Ba));
  EXPECT_TRUE(c.should_emit_for(CorrectionSource::TagLoop));
}
