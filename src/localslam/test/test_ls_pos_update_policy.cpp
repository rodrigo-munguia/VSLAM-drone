#include <gtest/gtest.h>

#include <limits>

#include "ls_pos_update_policy.hpp"

using localslam::PosUpdateAdmitResult;
using localslam::PosUpdateRequestView;
using localslam::admit_pos_update;

static slam_identity::UuidBytes sid(uint8_t fill)
{
  slam_identity::UuidBytes u{};
  u.fill(fill);
  u[6] = 0x40;
  u[8] = 0x80;
  return u;
}

TEST(LsPosUpdatePolicy, AcceptsMatchingIdentity)
{
  auto session = sid(1);
  std::unordered_set<uint64_t> applied;
  PosUpdateRequestView req;
  req.delta_x = 0.01;
  req.type = "bundle_adjustment";
  req.session_id = session;
  req.epoch_id = 2;
  req.keyframe_id = 5;
  req.correction_id = 9;
  auto r = admit_pos_update(req, true, session, 2, applied);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(applied.count(9u), 1u);
}

TEST(LsPosUpdatePolicy, RejectsStaleEpochAndSession)
{
  auto session = sid(1);
  auto other = sid(2);
  std::unordered_set<uint64_t> applied;
  PosUpdateRequestView req;
  req.delta_x = 0.01;
  req.type = "close_loop";
  req.session_id = session;
  req.epoch_id = 1;
  req.correction_id = 1;
  EXPECT_FALSE(admit_pos_update(req, true, session, 2, applied).accepted);
  req.epoch_id = 2;
  req.session_id = other;
  req.correction_id = 2;
  EXPECT_EQ(admit_pos_update(req, true, session, 2, applied).reason, "session_mismatch");
}

TEST(LsPosUpdatePolicy, RejectsInvalidRefDuplicateNonFiniteUnknown)
{
  auto session = sid(3);
  std::unordered_set<uint64_t> applied;
  PosUpdateRequestView req;
  req.delta_x = 0.01;
  req.type = "bundle_adjustment";
  req.session_id = session;
  req.epoch_id = 1;
  req.correction_id = 7;
  EXPECT_EQ(admit_pos_update(req, false, session, 1, applied).reason, "invalid_reference");
  EXPECT_TRUE(admit_pos_update(req, true, session, 1, applied).accepted);
  EXPECT_EQ(admit_pos_update(req, true, session, 1, applied).reason, "duplicate_correction_id");

  req.correction_id = 8;
  req.delta_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(admit_pos_update(req, true, session, 1, applied).reason, "non_finite_delta");
  req.delta_x = 0.01;
  req.type = "nope";
  req.correction_id = 9;
  EXPECT_EQ(admit_pos_update(req, true, session, 1, applied).reason, "unknown_type");
}

TEST(LsPosUpdatePolicy, RejectsZeroDeltaAndBookkeepingFull)
{
  auto session = sid(4);
  std::unordered_set<uint64_t> applied;
  PosUpdateRequestView req;
  req.type = "bundle_adjustment";
  req.session_id = session;
  req.epoch_id = 1;
  req.correction_id = 1;
  EXPECT_EQ(admit_pos_update(req, true, session, 1, applied).reason, "zero_delta");

  req.delta_x = 0.01;
  for (uint64_t i = 1; i <= localslam::kMaxAppliedCorrectionIds; ++i) {
    req.correction_id = i;
    ASSERT_TRUE(admit_pos_update(req, true, session, 1, applied).accepted)
      << "i=" << i;
  }
  req.correction_id = localslam::kMaxAppliedCorrectionIds + 1;
  EXPECT_EQ(
    admit_pos_update(req, true, session, 1, applied).reason,
    "correction_bookkeeping_full");
}
