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
  typedef std::shared_ptr<LinearEqualityConstraint> ConstraintPtr;

protected:
  virtual void SetUp(){

    subject_.Init(n_coeff_, n_steps_);

    int n_spline_coeff = subject_.GetSplineCoefficients().rows();
    InitialAccelerationEquation eq_acc_zeros(Eigen::Vector2d::Zero(),n_spline_coeff);
    InitialAccelerationEquation eq_acc_ones(Eigen::Vector2d::Ones(), n_spline_coeff);

    // use shared pointer to store constraint objects on heap
    c_zeros = std::make_shared<LinearEqualityConstraint>(subject_);
    c_ones  = std::make_shared<LinearEqualityConstraint>(subject_);

    c_zeros->Init(eq_acc_zeros.BuildLinearEquation());
    c_ones->Init(eq_acc_ones.BuildLinearEquation());

    constraints.AddConstraint(c_zeros, "zero_acc");
    constraints.AddConstraint(c_ones, "one_acc");
    constraints.RefreshBounds();
  }

  const int n_coeff_ = utils::kDim2d*4/*coefficients a,b,c,d*/;
  const int n_steps_ = 2;
  OptimizationVariables subject_;

  ConstraintPtr c_zeros, c_ones;
  ConstraintContainer constraints;
};

TEST_F(ConstraintContainerTest, EvaluateConstraints)
{
  Eigen::VectorXd g = constraints.EvaluateConstraints();

  EXPECT_EQ(4, g.rows()); // two constraints in x and one in y

  // remember: constraints stored in std::map, so no ordering (e.g. not first added first out)
  EXPECT_EQ(c_zeros->EvaluateConstraint(), g.tail<2>()); // two constraints in x and one in y
  EXPECT_EQ(c_ones->EvaluateConstraint() , g.head<2>()); // two constraints in x and one in y
}

TEST_F(ConstraintContainerTest, GetBounds)
{
  ConstraintContainer::VecBound bounds = constraints.GetBounds();

  EXPECT_EQ(4, bounds.size()); // two constraints in x and one in y

  for (auto b : constraints.GetConstraint("zero_acc").GetBounds()) {
    EXPECT_EQ(0.0, b.lower_);
    EXPECT_EQ(0.0, b.upper_);
  }

  for (auto b : constraints.GetConstraint("one_acc").GetBounds()) {
    EXPECT_EQ(1.0, b.lower_);
    EXPECT_EQ(1.0, b.upper_);
  }

}

} /* namespace zmp */
} /* namespace xpp */
