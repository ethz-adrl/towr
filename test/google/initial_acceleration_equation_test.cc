/**
 @file    initial_acceleration_equation_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/zmp/a_linear_constraint.h>
#include <xpp/zmp/initial_acceleration_equation.h>
#include <xpp/zmp/constraint_container.h>
#include <xpp/zmp/continuous_spline_container.h>

#include <gtest/gtest.h>

namespace xpp {
namespace zmp {

class InitialAccelerationEquationTest : public ::testing::Test {

public:
  InitialAccelerationEquationTest()
      : constraint_container_(subject_) {} // because these members have no default constructor

protected:
  virtual void SetUp()
  {
    subject_.Init(n_coeff_, n_steps_);
    init_acceleration_ << 1.3, 2.4; // x and y
    InitialAccelerationEquation eq(init_acceleration_, subject_.GetSplineCoefficients().rows());
    constraint_.Init(eq.BuildLinearEquation());
    EXPECT_EQ(1, subject_.GetObserverCount());
  }


  const int n_coeff_ = utils::kDim2d*kFreeCoeffPerSpline;
  const int n_steps_ = 2;
  Eigen::Vector2d init_acceleration_;
  OptimizationVariables subject_;
  LinearEqualityConstraint constraint_;
  ConstraintContainer constraint_container_;
};

TEST_F(InitialAccelerationEquationTest, EvaluateConstraint)
{
  Eigen::VectorXd g;
  // the splines are initialized with zero, so constraint represents this zero acceleration
  g = constraint_.EvaluateConstraint();
  EXPECT_EQ(2, g.rows());
  EXPECT_DOUBLE_EQ(0.0, g(0));
  EXPECT_DOUBLE_EQ(0.0, g(1));

  // change the splines acceleration
  // with new optimization variables the x-acceleration should cancel out.
  Eigen::VectorXd x(subject_.GetSplineCoefficients().rows());
  x.setZero();
  int idx = ContinuousSplineContainer::Index(0, utils::X, D);
  x(idx) = init_acceleration_.x()/2.0;

  subject_.SetSplineCoefficients(x);
  g = constraint_.EvaluateConstraint();
  EXPECT_EQ(2, g.rows());
  EXPECT_DOUBLE_EQ(init_acceleration_.x(), g(0));
  EXPECT_DOUBLE_EQ(0.0, g(1)); // same as before
}

TEST_F(InitialAccelerationEquationTest, GetBounds)
{
  AConstraint::VecBound bounds = constraint_.GetBounds();

  EXPECT_EQ(2, bounds.size()); // x and y acceleration
  // these are equality constraints
  EXPECT_DOUBLE_EQ(init_acceleration_.x(), bounds.at(0).lower_);
  EXPECT_DOUBLE_EQ(init_acceleration_.x(), bounds.at(0).upper_);
  EXPECT_DOUBLE_EQ(init_acceleration_.y(), bounds.at(1).lower_);
  EXPECT_DOUBLE_EQ(init_acceleration_.y(), bounds.at(1).upper_);
}


} /* namespace zmp */
} /* namespace xpp */
