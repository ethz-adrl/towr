/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/hyq/hyq_spliner.h>

#include <xpp/zmp/nlp_optimizer.h>


#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace hyq {


// A start and an end position for the splines. checking only boundary conditions
class HyqSplinerTest : public ::testing::Test {
public:
  typedef xpp::zmp::NlpOptimizer NlpOptimizer;
  typedef xpp::utils::Point2d Point2d;
  typedef xpp::utils::Point3d Point3d;

protected:
  virtual void SetUp()
  {
    // FIXME only for zmp publisher, remove from lowlevel optimizer code...
    int argc = 0;
    char **argv;
    ::ros::init(argc, argv, "HyqSplinerTest");

    NlpOptimizer nlp_optimizer;

    init_.feet_[LH].p << -0.31,  0.37, 0.0;
    init_.feet_[LF].p <<  0.33,  0.35, 0.0;
    init_.feet_[RH].p << -0.35, -0.33, 0.0;
    init_.feet_[RF].p <<  0.37, -0.31, 0.0;


    init_.base_.pos.p << 0.01, 0.4,  0.7;
    init_.base_.pos.v << 0.02, 0.5,  0.8;
    init_.base_.pos.a << 0.03, 0.6,  0.9;

    goal_.p <<  0.10, 0.11;
    goal_.v <<  0.12, 0.13;
    goal_.a <<  0.14, 0.15;

    std::vector<Foothold> start_stance;
    start_stance.push_back(Foothold(-0.31,  0.37, 0.0, LH));
    start_stance.push_back(Foothold( 0.33,  0.35, 0.0, LF));
    start_stance.push_back(Foothold(-0.35, -0.33, 0.0, RH));
    start_stance.push_back(Foothold( 0.37, -0.31, 0.0, RF));


    times_.t_stance_ = 0.2;
    times_.t_swing_ = 0.7;
    times_.t_stance_initial_ = 0.5; // this will create two initial splines
    times_.t_stance_final_ = 0.2;

    NlpOptimizer::VecSpline opt_xy_splines;
    NlpOptimizer::VecFoothold opt_footholds;
    robot_height_ = 0.58;
    nlp_optimizer.SolveNlp(init_.base_.pos.Get2D(), goal_,
                           {LH, LF, RH, RF}, init_.FeetToFootholds().ToVector(),
                           times_, robot_height_,
                           opt_xy_splines, opt_footholds);

    double upswing_percent = 0.5;
    double lift_height = 0.3;
    double outward_swing = 0.3;
    spliner_.SetParams(upswing_percent, lift_height, outward_swing);
    spliner_.Init(init_, opt_xy_splines, opt_footholds, robot_height_);
  }

  xpp::zmp::SplineTimes times_;
  HyqState init_;
  Point2d goal_;
  HyqSpliner spliner_;
  double robot_height_;
};


TEST_F(HyqSplinerTest, GetTotalTime)
{
  double T = times_.t_stance_initial_ + 4*times_.t_swing_ + times_.t_stance_ + times_.t_stance_final_;
  EXPECT_DOUBLE_EQ(T, spliner_.GetTotalTime());
}


TEST_F(HyqSplinerTest, GetSplineID)
{
  // remember that depending on the initial spline time, more than one
  // initial and final spline are created
  EXPECT_EQ(0, spliner_.GetSplineID(0.0));
  EXPECT_EQ(0, spliner_.GetSplineID(0.2));
  EXPECT_EQ(1, spliner_.GetSplineID(0.3));

  EXPECT_EQ(2, spliner_.GetSplineID(0.6));
  EXPECT_EQ(3, spliner_.GetSplineID(0.5+0.7+0.1));
}


TEST_F(HyqSplinerTest, GetGoalNode)
{
  // two splines to create inital movement
  EXPECT_DOUBLE_EQ(times_.t_stance_initial_/2.0, spliner_.GetGoalNode(0.0).T);
  EXPECT_DOUBLE_EQ(times_.t_stance_initial_/2.0, spliner_.GetGoalNode(0.3).T);
}


TEST_F(HyqSplinerTest, BuildNode)
{
  double t = 1.5;
  SplineNode node = HyqSpliner::BuildNode(init_, 1.5);

  EXPECT_DOUBLE_EQ(node.T, t);
  EXPECT_EQ(node.state_.base_.pos.p, init_.base_.pos.p);
  EXPECT_EQ(node.state_.base_.pos.v, init_.base_.pos.v);
  EXPECT_EQ(node.state_.base_.pos.a, init_.base_.pos.a);
  EXPECT_EQ(node.ori_rpy_.p, HyqSpliner::TransformQuatToRpy(init_.base_.ori.q));
}


TEST_F(HyqSplinerTest, GetCurrPositionInit)
{
  Point3d splined_init = spliner_.GetCurrPosition(0.0);
  SCOPED_TRACE(testing::Message() << "init_.base_.pos: " << init_.base_.pos);
  SCOPED_TRACE(testing::Message() << "splined_init: "    << splined_init);

  EXPECT_EQ(init_.base_.pos.p, splined_init.p);
  EXPECT_EQ(init_.base_.pos.v, splined_init.v);
  // these are constraints in the optimizer, so they won't be exactly fullfilled
  // only according to the "tol" parameter set in the ipopt config file
  double tol = 0.01; // m/s^2
  EXPECT_NEAR     (init_.base_.pos.a.x(), splined_init.a.x(), tol);
  EXPECT_NEAR     (init_.base_.pos.a.y(), splined_init.a.y(), tol);
  EXPECT_DOUBLE_EQ(init_.base_.pos.a.z(), splined_init.a.z());
}


TEST_F(HyqSplinerTest, GetCurrPositionGoal)
{
  double T = spliner_.GetTotalTime();
  Point3d splined_final = spliner_.GetCurrPosition(T);
  SCOPED_TRACE(testing::Message() << "goal_: " << goal_);
  SCOPED_TRACE(testing::Message() << "splined_final: "    << splined_final);

  // these are all constraints in the optimizer, so they won't be exactly fullfilled
  // only according to the "tol" parameter set in the ipopt config file
  double tol = 0.01; // m/s^2
  EXPECT_NEAR(goal_.p.x(), splined_final.p.x(), tol);
  EXPECT_NEAR(goal_.v.x(), splined_final.v.x(), tol);
  EXPECT_NEAR(goal_.a.x(), splined_final.a.x(), tol);

  EXPECT_NEAR(goal_.p.y(), splined_final.p.y(), tol);
  EXPECT_NEAR(goal_.v.y(), splined_final.v.y(), tol);
  EXPECT_NEAR(goal_.a.y(), splined_final.a.y(), tol);

  // z position is not optimized, and therefore matched exactly
  EXPECT_DOUBLE_EQ(robot_height_, splined_final.p.z());
  EXPECT_DOUBLE_EQ(0.0,           splined_final.v.z());
  EXPECT_DOUBLE_EQ(0.0,           splined_final.a.z());
}



} // namespace hyq
} // namespace xpp
