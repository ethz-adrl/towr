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
    int argc = 1;
    char *argv[] = {"dummy"};
    ::ros::init(argc, argv, "HyqSplinerTest");

    NlpOptimizer nlp_optimizer;

    init_.feet_[LH].p << -0.31,  0.37, 0.0;
    init_.feet_[LF].p <<  0.33,  0.35, 0.0;
    init_.feet_[RH].p << -0.35, -0.33, 0.0;
    init_.feet_[RF].p <<  0.37, -0.31, 0.0;


    goal_.p.x() = 0.25;

    std::vector<Foothold> start_stance;
    start_stance.push_back(Foothold(-0.31,  0.37, 0.0, LH));
    start_stance.push_back(Foothold( 0.33,  0.35, 0.0, LF));
    start_stance.push_back(Foothold(-0.35, -0.33, 0.0, RH));
    start_stance.push_back(Foothold( 0.37, -0.31, 0.0, RF));


    times_.t_stance_ = 0.2;
    times_.t_swing_ = 0.7;
    times_.t_stance_initial_ = 0.5; // this will create two initial splines
    times_.t_stance_final_ = 0.2;

    NlpOptimizer::VecSpline opt_splines;
    NlpOptimizer::VecFoothold opt_footholds;
    double robot_height = 0.58;
    nlp_optimizer.SolveNlp(init_.base_.pos.Get2D(), goal_,
                           {LH, LF, RH, RF}, init_.FeetToFootholds().ToVector(),
                           times_, robot_height,
                           opt_splines, opt_footholds);

    double upswing_percent = 0.5;
    double lift_height = 0.3;
    double outward_swing = 0.3;
    spliner_.SetParams(upswing_percent, lift_height, outward_swing);
    spliner_.Init(init_, opt_splines, opt_footholds, robot_height);

  }

  xpp::zmp::SplineTimes times_;
  HyqState init_;
  Point2d goal_;
  HyqSpliner spliner_;
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


TEST_F(HyqSplinerTest, GetSplinedState)
{
  HyqState initial_state = spliner_.GetSplinedState(0.0);
  EXPECT_EQ(init_.base_.pos.p, initial_state.base_.pos.p);

//  // not filling the position in code when constructing nodes, so this won't work
//  // zmp overwrites it anyways
//  HyqState inter_state = spliner_.GetSplinedState(0.5);
//  EXPECT_DOUBLE_EQ(goal_.p.x(), inter_state.base_.pos.p.x());
//  EXPECT_DOUBLE_EQ(goal_.p.y(), inter_state.base_.pos.p.y());
//
//
//  HyqState final_state = spliner_.GetSplinedState(spliner_.GetTotalTime());
//  EXPECT_DOUBLE_EQ(goal_.p.x(), final_state.base_.pos.p.x());
//  EXPECT_DOUBLE_EQ(goal_.p.y(), final_state.base_.pos.p.y());
}







































} // namespace hyq
} // namespace xpp
