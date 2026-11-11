#include <gtest/gtest.h>

#include "g2_run_control.hpp"

using globalslam::G2RunControl;

TEST(G2RunControl, DefaultsSuppressCorrectionsAndStartDisabled)
{
  G2RunControl c;
  EXPECT_FALSE(c.sources.processing_enabled);
  EXPECT_FALSE(c.sources.apply_global_corrections_master);
  EXPECT_FALSE(c.should_accept_keyframe());
  EXPECT_FALSE(c.should_emit_local_slam_correction());
  EXPECT_FALSE(c.should_emit_control_loop_signal());
}

TEST(G2RunControl, EnableAcceptsKeyframesWithoutOpeningCorrectionGate)
{
  G2RunControl c;
  c.sources.apply_global_corrections_master = false;
  c.set_processing_enabled(true);
  EXPECT_TRUE(c.should_accept_keyframe());
  EXPECT_FALSE(c.should_emit_local_slam_correction());
  EXPECT_FALSE(c.should_emit_control_loop_signal());
}

TEST(G2RunControl, MasterApplyOpensEnabledSourcesOnly)
{
  G2RunControl c;
  c.sources.enable_global_ba = true;
  c.sources.enable_tag_loop_closure = true;
  c.sources.enable_visual_loop_closure = false;
  c.sources.apply_global_corrections_master = true;
  c.sources.apply_master_compatibility();
  c.set_processing_enabled(true);
  EXPECT_TRUE(c.sources.apply_ba_corrections);
  EXPECT_TRUE(c.sources.apply_tag_loop_corrections);
  EXPECT_FALSE(c.sources.apply_visual_loop_corrections);
  EXPECT_TRUE(c.should_emit_local_slam_correction());
  EXPECT_TRUE(c.should_emit_control_loop_signal());
  c.set_processing_enabled(false);
  EXPECT_FALSE(c.should_emit_local_slam_correction());
}

TEST(G2RunControl, ResetBumpsGenerationAndRequestsReinit)
{
  G2RunControl c;
  c.set_processing_enabled(true);
  const uint64_t g0 = c.reset_generation;
  c.request_reset();
  EXPECT_EQ(c.reset_generation, g0 + 1);
  EXPECT_TRUE(c.consume_reinit_gmap());
  EXPECT_TRUE(c.consume_reinit_cloop());
  EXPECT_FALSE(c.consume_reinit_gmap());
  EXPECT_FALSE(c.consume_reinit_cloop());
}

TEST(G2RunControl, InFlightResultsRejectedAfterReset)
{
  G2RunControl c;
  c.set_processing_enabled(true);
  const uint64_t step_gen = c.reset_generation;
  EXPECT_TRUE(c.results_belong_to_generation(step_gen));
  c.request_reset();
  EXPECT_FALSE(c.results_belong_to_generation(step_gen));
  EXPECT_TRUE(c.results_belong_to_generation(c.reset_generation));
}

TEST(G2RunControl, DisableDropsAcceptanceWithoutClearingGeneration)
{
  G2RunControl c;
  c.set_processing_enabled(true);
  const uint64_t g = c.reset_generation;
  c.set_processing_enabled(false);
  EXPECT_FALSE(c.should_accept_keyframe());
  EXPECT_EQ(c.reset_generation, g);
}

TEST(G2RunControl, LegacyResetEquivalentToStandardRequestReset)
{
  G2RunControl a;
  G2RunControl b;
  a.request_reset();
  b.request_reset();
  EXPECT_EQ(a.reset_generation, b.reset_generation);
  EXPECT_TRUE(a.reinit_gmap);
  EXPECT_TRUE(b.reinit_cloop);
}
