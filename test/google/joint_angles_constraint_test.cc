/**
 @file    joint_angles_constraint_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/opt/joint_angles_constraint.h>
#include <xpp/opt/constraint_container.h>
#include <xpp/opt/cost_constraint_factory.h>
#include <xpp/opt/optimization_variables.h>

#include <gtest/gtest.h>

namespace xpp {
namespace opt {

TEST(JointAnglesContraintTest, StartStanceInLimits)
{
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

  // create the framework of the optimization
  OptimizationVariablesInterpreter interpreter;
  ComSpline4 spline_structure;
  spline_structure.Init(init_pos, init_vel, 0, times, true);
  hyq::SupportPolygonContainer support_polygon_container_;
  support_polygon_container_.Init(start_stance_, hyq::SupportPolygonContainer::VecFoothold(), hyq::SupportPolygon::GetZeroMargins());
  interpreter.Init(spline_structure, support_polygon_container_, robot_height);

  // create the joint angle constraint and put into the container
  int n_splines = 1;
  OptimizationVariablesContainer opt_variables;
  opt_variables.AddVariableSet(0, Eigen::VectorXd(n_splines*kFreeCoeffPerSpline*2));
  ConstraintContainer constraint_container(opt_variables);
  auto constraint = CostConstraintFactory::CreateJointAngleConstraint(interpreter);
  constraint_container.AddConstraint(constraint);

  VectorXd q = constraint_container.GetConstraintValues();
  Constraint::VecBound q_bounds = constraint_container.GetBounds();

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
