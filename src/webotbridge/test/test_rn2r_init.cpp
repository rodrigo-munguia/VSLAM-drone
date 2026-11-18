#include <gtest/gtest.h>
#include <armadillo>

#include "rn2r_init.hpp"

// Documents the Armadillo pitfall that caused CTRL-S3b container death:
// bare eye() does not allocate 3×3 on a default-constructed empty mat.
TEST(Rn2rInit, BareEyeOnEmptyMatStaysZeroByZero)
{
  arma::mat A;
  ASSERT_EQ(A.n_rows, 0u);
  ASSERT_EQ(A.n_cols, 0u);
  A.eye();  // no-op on empty — stays 0×0
  EXPECT_EQ(A.n_rows, 0u);
  EXPECT_EQ(A.n_cols, 0u);
  EXPECT_FALSE(webotbridge::rn2rg_is_3x3(A));
}

TEST(Rn2rInit, InitIdentityIs3x3AndMultiplies)
{
  arma::mat Rn2rG;
  webotbridge::init_rn2rg_identity(Rn2rG);
  ASSERT_TRUE(webotbridge::rn2rg_is_3x3(Rn2rG));
  arma::vec v = {1.0, 2.0, 3.0};
  arma::vec r = Rn2rG * v;
  EXPECT_DOUBLE_EQ(r(0), 1.0);
  EXPECT_DOUBLE_EQ(r(1), 2.0);
  EXPECT_DOUBLE_EQ(r(2), 3.0);
}

TEST(Rn2rInit, GuardRepairsEmptyBeforeMultiply)
{
  arma::mat Rn2rG;  // 0x0 as after buggy eye()
  if (!webotbridge::rn2rg_is_3x3(Rn2rG)) {
    webotbridge::init_rn2rg_identity(Rn2rG);
  }
  arma::vec sp_N = {0.5, -0.25, 0.1};
  arma::vec sp_R = Rn2rG * sp_N;
  EXPECT_DOUBLE_EQ(sp_R(0), 0.5);
  EXPECT_DOUBLE_EQ(sp_R(1), -0.25);
  EXPECT_DOUBLE_EQ(sp_R(2), 0.1);
}
