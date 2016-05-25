/*
 * initial_acceleration_constraint_test.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/initial_acceleration_constraint.h>
#include <gtest/gtest.h>

namespace xpp {
namespace zmp {

class InitialAccelerationConstraintTest : public ::testing::Test {

public:
  InitialAccelerationConstraintTest()
      : subject_(3*utils::kDim2d*kFreeCoeffPerSpline, 0),
        constraint_(subject_) {} // because these members have no default constructor

protected:
  virtual void SetUp()
  {
    init_acceleration_ << 1.3, 2.4; // x and y
    constraint_.SetDesiredInitialAcceleration(init_acceleration_);

    EXPECT_EQ(1, subject_.GetObserverCount());
  }

  Eigen::Vector2d init_acceleration_;
  OptimizationVariables subject_;
  InitialAccelerationConstraint constraint_;
};

TEST_F(InitialAccelerationConstraintTest, EvaluateConstraint)
{
  Eigen::VectorXd g;
  // the splines are initialized with zero, so constraint violation will be negative desired acceleration
  g = constraint_.EvaluateConstraint();
  EXPECT_EQ(2, g.rows());
  EXPECT_DOUBLE_EQ(-init_acceleration_.x(), g(0));
  EXPECT_DOUBLE_EQ(-init_acceleration_.y(), g(1));

  // change the splines acceleration
  int n_opt_var = subject_.GetOptimizationVariableCount();
  Eigen::VectorXd x(n_opt_var);
  x.setZero();
  int idx = ContinuousSplineContainer::Index(0, utils::X, D);
  x(idx) = init_acceleration_.x()/2.0;

  subject_.SetVariables(x);
  g = constraint_.EvaluateConstraint();
  EXPECT_EQ(2, g.rows());
  EXPECT_DOUBLE_EQ(0.0, g(0));
  EXPECT_DOUBLE_EQ(-init_acceleration_.y(), g(1));
}

TEST_F(InitialAccelerationConstraintTest, GetBounds)
{
  AConstraint::VecBound bounds = constraint_.GetBounds();

  EXPECT_EQ(2, bounds.size()); // x and y acceleration
  // these are equality constraints
  EXPECT_DOUBLE_EQ(0.0, bounds.at(0).lower_);
  EXPECT_DOUBLE_EQ(0.0, bounds.at(0).upper_);
  EXPECT_DOUBLE_EQ(0.0, bounds.at(1).lower_);
  EXPECT_DOUBLE_EQ(0.0, bounds.at(1).upper_);
}


} /* namespace zmp */
} /* namespace xpp */
