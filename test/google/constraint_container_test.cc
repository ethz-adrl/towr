/*
 * constraint_container_test.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraint_container.h>
#include <xpp/zmp/optimization_variables.h>

#include <xpp/zmp/a_linear_constraint.h>
#include <xpp/zmp/initial_acceleration_equation.h>

#include <gtest/gtest.h>

namespace xpp {
namespace zmp {


class ConstraintContainerTest : public ::testing::Test {
public:
  ConstraintContainerTest()
      : subject_(n_coeff_, n_steps_),
        c_zeros(subject_),
        c_ones(subject_)
  {}

protected:
  virtual void SetUp(){

    int n_spline_coeff = subject_.GetSplineCoefficients().rows();
    InitialAccelerationEquation eq_acc_zeros(Eigen::Vector2d::Zero(),n_spline_coeff);
    InitialAccelerationEquation eq_acc_ones(Eigen::Vector2d::Ones(), n_spline_coeff);

    c_zeros.Init(eq_acc_zeros.BuildLinearEquation());
    c_ones.Init(eq_acc_ones.BuildLinearEquation());


    constraints.AddConstraint(c_zeros);
    constraints.AddConstraint(c_ones);
  }

  const int n_coeff_ = utils::kDim2d*kFreeCoeffPerSpline;
  const int n_steps_ = 2;
  OptimizationVariables subject_;

  LinearEqualityConstraint c_zeros, c_ones;
  ConstraintContainer constraints;
};

TEST_F(ConstraintContainerTest, EvaluateConstraints)
{
  Eigen::VectorXd g = constraints.EvaluateConstraints();

  EXPECT_EQ(4, g.rows()); // two constraints in x and one in y
  EXPECT_EQ(c_zeros.EvaluateConstraint(), g.head<2>()); // two constraints in x and one in y
  EXPECT_EQ(c_ones.EvaluateConstraint() , g.tail<2>()); // two constraints in x and one in y
}

TEST_F(ConstraintContainerTest, GetBounds)
{
  ConstraintContainer::VecBound bounds = constraints.GetBounds();

  EXPECT_EQ(4, bounds.size()); // two constraints in x and one in y
  for (AConstraint::Bound b : bounds) {
    EXPECT_EQ(0.0, b.lower_);
    EXPECT_EQ(0.0, b.upper_);
  }

}

} /* namespace zmp */
} /* namespace xpp */
