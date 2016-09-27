/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include "nlp_facade_test.h"


namespace xpp {
namespace zmp {

// initializing the static member variables
SplineTimes NlpFacadeTest::times_ = SplineTimes();
NlpFacadeTest::Point2d  NlpFacadeTest::start_xy_ = Point2d();
NlpFacadeTest::Point2d  NlpFacadeTest::goal_xy_  = Point2d();
NlpFacadeTest::VecFoothold NlpFacadeTest::start_stance_ = VecFoothold();
NlpFacadeTest::VecSpline NlpFacadeTest::opt_xy_splines_ = VecSpline();
NlpFacadeTest::VecFoothold NlpFacadeTest::opt_footholds_ = VecFoothold();


TEST_F(NlpFacadeTest, GetCOGxyInitialAccelerationConstraint)
{
  double t0 = 0.0;
  xpp::utils::BaseLin2d xy_optimized = ComSpline6::GetCOGxy(t0, opt_xy_splines_);

  // these are hardcoded into the splines
  EXPECT_EQ(start_xy_.p, xy_optimized.p);
  EXPECT_EQ(start_xy_.v, xy_optimized.v);
  // these are constraints in the optimizer, so they won't be exactly fullfilled
  // only according to the "tol" parameter set in the ipopt config file
  double tol = 0.01; // m/s^2
  EXPECT_NEAR     (start_xy_.a.x(), xy_optimized.a.x(), tol);
  EXPECT_NEAR     (start_xy_.a.y(), xy_optimized.a.y(), tol);
}


TEST_F(NlpFacadeTest, GetCOGxyFinalAccelerationConstraint)
{
  double T = ComSpline6::GetTotalTime(opt_xy_splines_);
  xpp::utils::BaseLin2d xy_optimized = ComSpline6::GetCOGxy(T, opt_xy_splines_);

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
