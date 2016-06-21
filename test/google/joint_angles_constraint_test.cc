/**
 @file    joint_angles_constraint_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/zmp/joint_angles_constraint.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>

#include <gtest/gtest.h>

namespace xpp {
namespace zmp {

TEST(JointAnglesContraintTest, StartStanceInLimits)
{
  int n_splines = 1;
  OptimizationVariables subject_(n_splines*kFreeCoeffPerSpline*2, 0);

  typedef Eigen::Vector2d Vector2d;
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::hyq::Foothold Foothold;
  Vector2d init_pos, init_vel;
  init_pos.setZero(); init_vel.setZero();

  std::vector<Foothold> start_stance_;
  start_stance_.push_back(Foothold(-0.31,  0.37, 0.0, xpp::hyq::LH));
  start_stance_.push_back(Foothold( 0.33,  0.35, 0.0, xpp::hyq::LF));
  start_stance_.push_back(Foothold(-0.35, -0.33, 0.0, xpp::hyq::RH));
  start_stance_.push_back(Foothold( 0.37, -0.31, 0.0, xpp::hyq::RF));

  double robot_height = 0.58;
  SplineTimes times;
  times.SetDefault();

  OptimizationVariablesInterpreter interpreter;
  ContinuousSplineContainer spline_structure;
  spline_structure.Init(init_pos, init_vel, 0, times, true, false);
  interpreter.Init(spline_structure, {}, start_stance_, robot_height);


  xpp::hyq::HyqInverseKinematics hyq_inv_kin;

  JointAnglesConstraint constraint(subject_);
  constraint.Init(interpreter, &hyq_inv_kin);

  VectorXd q = constraint.EvaluateConstraint();
  AConstraint::VecBound q_bounds = constraint.GetBounds();


  EXPECT_GT(q.rows(), 0);
  EXPECT_GT(q_bounds.size(), 0);
  EXPECT_EQ(q.rows(), q_bounds.size());


  std::cout << std::setprecision(2);
  for (int i=0; i<q.rows(); ++i) {
    EXPECT_LT(q_bounds.at(i).lower_, q[i]);
    EXPECT_LT(q[i], q_bounds.at(i).upper_);
  }


}

} /* namespace zmp */
} /* namespace xpp */
