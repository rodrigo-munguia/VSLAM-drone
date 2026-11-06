#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "pressure_altitude.hpp"

using localslam::pressure_altitude::ConvertConfig;
using localslam::pressure_altitude::PressureToRelativeAltitude;
using localslam::pressure_altitude::isa_height_m_from_pressure_pa;
using localslam::pressure_altitude::isa_pressure_pa_from_height_m;
using localslam::pressure_altitude::kP0_Pa;
using localslam::pressure_altitude::kT0_K;

static sensor_msgs::msg::FluidPressure make_fp(
  double pa, const std::string & frame = "baro_link", double variance = 1.0)
{
  sensor_msgs::msg::FluidPressure msg;
  msg.header.frame_id = frame;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nanosec = 0;
  msg.fluid_pressure = pa;
  msg.variance = variance;  // must be ignored by converter
  return msg;
}

TEST(IsaModel, SeaLevelPressureIsZeroHeight)
{
  const double h = isa_height_m_from_pressure_pa(kP0_Pa);
  ASSERT_TRUE(std::isfinite(h));
  EXPECT_NEAR(h, 0.0, 1e-6);
}

TEST(IsaModel, PressureDropGivesPositiveAltitude)
{
  // ~1000 m ISA: P ≈ 89874.57 Pa (independent inverse formula)
  const double p_1000 = isa_pressure_pa_from_height_m(1000.0);
  ASSERT_TRUE(std::isfinite(p_1000));
  EXPECT_LT(p_1000, kP0_Pa);
  const double h = isa_height_m_from_pressure_pa(p_1000);
  EXPECT_NEAR(h, 1000.0, 0.05);
}

TEST(IsaModel, KnownReferenceCases)
{
  // Cross-check with published ISA round-trips (inverse ≠ forward code path).
  struct Case { double h_m; double tol_m; };
  const Case cases[] = {
    {0.0, 1e-6},
    {500.0, 0.05},
    {1500.0, 0.1},
    {3000.0, 0.2},
    {5000.0, 0.5},
  };
  for (const auto & c : cases) {
    const double p = isa_pressure_pa_from_height_m(c.h_m);
    ASSERT_TRUE(std::isfinite(p)) << c.h_m;
    const double h = isa_height_m_from_pressure_pa(p);
    EXPECT_NEAR(h, c.h_m, c.tol_m) << "h=" << c.h_m;
  }
}

TEST(IsaModel, InvalidPressure)
{
  EXPECT_TRUE(std::isnan(isa_height_m_from_pressure_pa(
    std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(isa_height_m_from_pressure_pa(-10.0)));
  EXPECT_TRUE(std::isnan(isa_height_m_from_pressure_pa(0.0)));
}

TEST(PressureConverter, ReferenceGivesZeroRelative)
{
  PressureToRelativeAltitude conv;
  ConvertConfig cfg;
  conv.configure(cfg);
  const double p_ref = isa_pressure_pa_from_height_m(1200.0);
  const auto r0 = conv.convert(make_fp(p_ref));
  ASSERT_TRUE(r0.ok) << r0.error;
  EXPECT_TRUE(r0.latched_new_reference);
  EXPECT_NEAR(r0.alt.altitude, 0.0, 1e-9);
  EXPECT_NEAR(r0.h_rel_m, 0.0, 1e-9);
}

TEST(PressureConverter, SignAndMagnitude)
{
  PressureToRelativeAltitude conv;
  conv.configure(ConvertConfig{});
  const double h0 = 800.0;
  const double dh = 25.0;
  const double p0 = isa_pressure_pa_from_height_m(h0);
  const double p_up = isa_pressure_pa_from_height_m(h0 + dh);
  ASSERT_TRUE(conv.convert(make_fp(p0)).ok);
  const auto up = conv.convert(make_fp(p_up));
  ASSERT_TRUE(up.ok) << up.error;
  EXPECT_GT(up.alt.altitude, 0.0);
  EXPECT_NEAR(up.alt.altitude, dh, 0.05);

  const double p_down = isa_pressure_pa_from_height_m(h0 - dh);
  const auto down = conv.convert(make_fp(p_down));
  ASSERT_TRUE(down.ok);
  EXPECT_LT(down.alt.altitude, 0.0);
  EXPECT_NEAR(down.alt.altitude, -dh, 0.05);
}

TEST(PressureConverter, IgnoresVarianceField)
{
  PressureToRelativeAltitude conv;
  conv.configure(ConvertConfig{});
  const double p = isa_pressure_pa_from_height_m(100.0);
  auto a = conv.convert(make_fp(p, "baro_link", 0.0));
  ASSERT_TRUE(a.ok);
  conv.reset_reference();
  auto b = conv.convert(make_fp(p, "baro_link", 1e6));
  ASSERT_TRUE(b.ok);
  EXPECT_DOUBLE_EQ(a.alt.altitude, b.alt.altitude);
}

TEST(PressureConverter, RejectsNonFiniteAndOutOfBand)
{
  PressureToRelativeAltitude conv;
  ConvertConfig cfg;
  cfg.p_min_Pa = 50000.0;
  cfg.p_max_Pa = 105000.0;
  conv.configure(cfg);
  EXPECT_FALSE(conv.convert(make_fp(
    std::numeric_limits<double>::quiet_NaN())).ok);
  EXPECT_FALSE(conv.convert(make_fp(20000.0)).ok);  // below min
  EXPECT_FALSE(conv.convert(make_fp(110000.0)).ok);  // above max
}

TEST(PressureConverter, ResetRelatchesReference)
{
  PressureToRelativeAltitude conv;
  conv.configure(ConvertConfig{});
  const double p1 = isa_pressure_pa_from_height_m(1000.0);
  const double p2 = isa_pressure_pa_from_height_m(1100.0);
  ASSERT_TRUE(conv.convert(make_fp(p1)).ok);
  auto mid = conv.convert(make_fp(p2));
  ASSERT_TRUE(mid.ok);
  EXPECT_NEAR(mid.alt.altitude, 100.0, 0.1);

  conv.reset_reference();
  EXPECT_FALSE(conv.has_reference());
  auto again = conv.convert(make_fp(p2));
  ASSERT_TRUE(again.ok);
  EXPECT_TRUE(again.latched_new_reference);
  EXPECT_NEAR(again.alt.altitude, 0.0, 1e-9);
}

TEST(PressureConverter, FrameAndLeverArm)
{
  PressureToRelativeAltitude conv;
  ConvertConfig cfg;
  cfg.expected_frame_id = "baro_link";
  cfg.z_lever_arm_m = 0.2;  // sensor 0.2 m above body origin
  conv.configure(cfg);
  EXPECT_FALSE(conv.convert(make_fp(kP0_Pa, "wrong")).ok);
  const auto r = conv.convert(make_fp(kP0_Pa, "baro_link"));
  ASSERT_TRUE(r.ok) << r.error;
  // Relative ISA is 0; lever arm subtracted → -0.2 m at body origin.
  EXPECT_NEAR(r.alt.altitude, -0.2, 1e-9);
}

TEST(PressureConverter, ConfiguredNoiseNotFromVariance)
{
  // Document/contract: converter never reads variance into AltObs.
  // Sigma_alt_update remains a separate ROS parameter (not part of helper).
  PressureToRelativeAltitude conv;
  conv.configure(ConvertConfig{});
  auto r = conv.convert(make_fp(kP0_Pa, "baro_link", 42.0));
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.alt.altitude, 0.0, 1e-9);
}
