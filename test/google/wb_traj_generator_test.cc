/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include "../../include/xpp/hyq/wb_traj_generator.h"

#include <gtest/gtest.h>
#include <iostream>
#include "nlp_facade_test.h"

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace hyq {


// uses the optimized spline created by the nlp optimizer
class HyqSplinerTest : public xpp::opt::NlpFacadeTest {
public:
  typedef xpp::utils::StateLin2d Point2d;
  typedef xpp::utils::StateLin3d Point3d;

protected:
  virtual void SetUp()
  {
    NlpFacadeTest::SetUp(); // initialize super class

    init_.base_.lin = start_xy_.Make3D();
    init_.base_.lin.p.z() = 0.7;
    init_.base_.lin.v.z() = 0.8;
    init_.base_.lin.a.z() = 0.9;

    for (LegID leg : kHyqLegOrder) {
      init_.feet_[leg].p = Foothold::GetLastFoothold(leg, start_stance_).p;
    }

    double upswing_percent = 0.5;
    double lift_height = 0.3;
    double outward_swing = 0.3;
    spliner_.SetParams(upswing_percent, lift_height, outward_swing);
    spliner_.Init(init_, opt_xy_splines_, opt_footholds_, robot_height_);
  }

  HyqState init_;
  WBTrajGenerator spliner_;
};

TEST_F(HyqSplinerTest, GetTotalTime)
{
  double T = times_.t_stance_initial_ + 2*times_.t_swing_  + times_.t_stance_final_;
  EXPECT_DOUBLE_EQ(T, spliner_.GetTotalTime());
}

TEST_F(HyqSplinerTest, GetSplineID)
{
  EXPECT_EQ(0, spliner_.GetSplineID(0.0));
  EXPECT_EQ(0, spliner_.GetSplineID(0.3));

  // here t>t_stance_initial
  EXPECT_EQ(1, spliner_.GetSplineID(times_.t_stance_initial_ + 0.01));
  EXPECT_EQ(2, spliner_.GetSplineID(times_.t_stance_initial_ + times_.t_swing_ + 0.01));
}

TEST_F(HyqSplinerTest, GetGoalNode)
{
  // same goal node while still in initial spline
  EXPECT_DOUBLE_EQ(times_.t_stance_initial_, spliner_.GetGoalNode(0.0).T);
  EXPECT_DOUBLE_EQ(times_.t_stance_initial_, spliner_.GetGoalNode(0.3).T);
}

TEST_F(HyqSplinerTest, BuildNode)
{
  double t = 1.5;
  SplineNode node = WBTrajGenerator::BuildNode(init_, 1.5);

  EXPECT_DOUBLE_EQ(node.T, t);
  EXPECT_EQ(node.state_.base_.lin.p, init_.base_.lin.p);
  EXPECT_EQ(node.state_.base_.lin.v, init_.base_.lin.v);
  EXPECT_EQ(node.state_.base_.lin.a, init_.base_.lin.a);
  EXPECT_EQ(node.ori_rpy_.p, WBTrajGenerator::TransformQuatToRpy(init_.base_.ang.q));
}

TEST_F(HyqSplinerTest, GetCurrPositionInit)
{
  Point3d splined_init = spliner_.GetCurrPosition(0.0);
  SCOPED_TRACE(testing::Message() << "init_.base_.pos: " << init_.base_.lin);
  SCOPED_TRACE(testing::Message() << "splined_init: "    << splined_init);

  // xy positions filled in by optimized spline and tested in NlpOptimizer
  EXPECT_DOUBLE_EQ(init_.base_.lin.a.z(), splined_init.a.z());
}

TEST_F(HyqSplinerTest, GetCurrPositionGoal)
{
  double T = spliner_.GetTotalTime();
  Point3d splined_final = spliner_.GetCurrPosition(T);
  SCOPED_TRACE(testing::Message() << "goal_: "         << goal_xy_);
  SCOPED_TRACE(testing::Message() << "splined_final: " << splined_final);

  // z position is not optimized, and therefore matched exactly
  EXPECT_DOUBLE_EQ(robot_height_, splined_final.p.z());
  EXPECT_DOUBLE_EQ(0.0,           splined_final.v.z());
  EXPECT_DOUBLE_EQ(0.0,           splined_final.a.z());
}

} // namespace hyq
} // namespace xpp
