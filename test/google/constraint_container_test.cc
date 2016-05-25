/*
 * constraint_container_test.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraint_container.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/initial_acceleration_constraint.h>

#include <gtest/gtest.h>

namespace xpp {
namespace zmp {


class ConstraintContainerTest : public ::testing::Test {
public:
  ConstraintContainerTest()
      : subject_(n_coeff_, n_steps_),
        c1(subject_),
        c2(subject_)
  {}

protected:
  virtual void SetUp(){
    c1.SetDesiredInitialAcceleration(Eigen::Vector2d::Ones());
    c2.SetDesiredInitialAcceleration(Eigen::Vector2d::Zero());

    constraints.AddConstraint(c1);
    constraints.AddConstraint(c2);
  }

  const int n_coeff_ = utils::kDim2d*kFreeCoeffPerSpline;
  const int n_steps_ = 2;
  OptimizationVariables subject_;

  InitialAccelerationConstraint c1;
  InitialAccelerationConstraint c2;
  ConstraintContainer constraints;
};

TEST_F(ConstraintContainerTest, EvaluateConstraints)
{
  Eigen::VectorXd g = constraints.EvaluateConstraints();

  EXPECT_EQ(4, g.rows()); // two constraints in x and one in y
  EXPECT_EQ(c1.EvaluateConstraint(), g.head<2>()); // two constraints in x and one in y
  EXPECT_EQ(c2.EvaluateConstraint(), g.tail<2>()); // two constraints in x and one in y
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
