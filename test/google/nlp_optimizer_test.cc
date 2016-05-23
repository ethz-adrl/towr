/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include "nlp_optimizer_test.h"


namespace xpp {
namespace zmp {

// initializing the static member variables
SplineTimes NlpOptimizerTest::times_ = SplineTimes();
NlpOptimizerTest::Point2d  NlpOptimizerTest::start_xy_ = Point2d();
NlpOptimizerTest::Point2d  NlpOptimizerTest::goal_xy_  = Point2d();
NlpOptimizerTest::VecFoothold NlpOptimizerTest::start_stance_ = VecFoothold();
NlpOptimizerTest::VecSpline NlpOptimizerTest::opt_xy_splines_ = VecSpline();
NlpOptimizerTest::VecFoothold NlpOptimizerTest::opt_footholds_ = VecFoothold();


TEST_F(NlpOptimizerTest, GetCOGxyInitialAccelerationConstraint)
{
  double t0 = 0.0;
  xpp::utils::Point2d xy_optimized = SplineContainer::GetCOGxy(t0, opt_xy_splines_);

  // these are hardcoded into the splines
  EXPECT_EQ(start_xy_.p, xy_optimized.p);
  EXPECT_EQ(start_xy_.v, xy_optimized.v);
  // these are constraints in the optimizer, so they won't be exactly fullfilled
  // only according to the "tol" parameter set in the ipopt config file
  double tol = 0.01; // m/s^2
  EXPECT_NEAR     (start_xy_.a.x(), xy_optimized.a.x(), tol);
  EXPECT_NEAR     (start_xy_.a.y(), xy_optimized.a.y(), tol);
}


TEST_F(NlpOptimizerTest, GetCOGxyFinalAccelerationConstraint)
{
  double T = SplineContainer::GetTotalTime(opt_xy_splines_);
  xpp::utils::Point2d xy_optimized = SplineContainer::GetCOGxy(T, opt_xy_splines_);

  // these are all constraints in the optimizer, so they won't be exactly fullfilled
  // only according to the "tol" parameter set in the ipopt config file
  double tol = 0.01; // m/s^2
  EXPECT_NEAR(goal_xy_.p.x(), xy_optimized.p.x(), tol);
  EXPECT_NEAR(goal_xy_.v.x(), xy_optimized.v.x(), tol);
  EXPECT_NEAR(goal_xy_.a.x(), xy_optimized.a.x(), tol);

  EXPECT_NEAR(goal_xy_.p.y(), xy_optimized.p.y(), tol);
  EXPECT_NEAR(goal_xy_.v.y(), xy_optimized.v.y(), tol);
  EXPECT_NEAR(goal_xy_.a.y(), xy_optimized.a.y(), tol);
}

} // namespace zmp
} // namespace xpp
