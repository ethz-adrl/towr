/*
 * constraint_container_test.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/constraint_container.h>
#include <xpp/zmp/constraint_factory.h>

#include <gtest/gtest.h>

namespace xpp {
namespace zmp {


class ConstraintContainerTest : public ::testing::Test {
public:
  typedef std::shared_ptr<ConstraintContainer> ConstraintContainerPtr;
  typedef std::shared_ptr<AConstraint> ConstraintPtr;

protected:
  virtual void SetUp()
  {
    const int n_coeff_ = utils::kDim2d*4 /*coefficients a,b,c,d*/;
    const int n_steps_ = 2;
    opt_var_.Init(n_coeff_, n_steps_);
    constraints_ = std::make_shared<ConstraintContainer>(opt_var_);

    int n_spline_coeff = opt_var_.GetSplineCoefficients().rows();
    c_zeros_ = ConstraintFactory::CreateAccConstraint(Eigen::Vector2d::Zero(),n_spline_coeff);
    c_ones_  = ConstraintFactory::CreateAccConstraint(Eigen::Vector2d::Ones(),n_spline_coeff);

    constraints_->AddConstraint(c_zeros_, "zero_acc");
    constraints_->AddConstraint(c_ones_, "one_acc");
  }

  OptimizationVariables opt_var_;
  ConstraintPtr c_zeros_, c_ones_;
  ConstraintContainerPtr constraints_;
};

TEST_F(ConstraintContainerTest, EvaluateConstraints)
{
  Eigen::VectorXd g = constraints_->EvaluateConstraints();
  std::cout << "g: " << g.transpose() << std::endl;

  EXPECT_EQ(4, g.rows()); // two constraints in x and one in y

  // remember: constraints stored in std::map, so no ordering (e.g. not first added first out)
  EXPECT_EQ(c_zeros_->EvaluateConstraint(), g.tail<2>()); // two constraints in x and one in y
  EXPECT_EQ(c_ones_->EvaluateConstraint() , g.head<2>()); // two constraints in x and one in y
}

TEST_F(ConstraintContainerTest, GetBounds)
{
  ConstraintContainer::VecBound bounds = constraints_->GetBounds();

  EXPECT_EQ(4, bounds.size()); // two constraints in x and one in y

  for (auto b : constraints_->GetConstraint("zero_acc").GetBounds()) {
    EXPECT_EQ(0.0, b.lower_);
    EXPECT_EQ(0.0, b.upper_);
  }

  for (auto b : constraints_->GetConstraint("one_acc").GetBounds()) {
    EXPECT_EQ(1.0, b.lower_);
    EXPECT_EQ(1.0, b.upper_);
  }

}

} /* namespace zmp */
} /* namespace xpp */
