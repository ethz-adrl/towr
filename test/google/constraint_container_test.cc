/*
 * constraint_container_test.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/opt/constraint_container.h>
#include <xpp/opt/com_spline4.h>
#include <xpp/opt/cost_constraint_factory.h>
#include <xpp/opt/optimization_variables.h>

#include <gtest/gtest.h>

namespace xpp {
namespace opt {

class ConstraintContainerTest : public ::testing::Test {
public:
  typedef std::shared_ptr<ConstraintContainer> ConstraintContainerPtr;
  typedef std::shared_ptr<Constraint> ConstraintPtr;

protected:
  virtual void SetUp()
  {
    const int n_coeff_ = utils::kDim2d*4 /*coefficients a,b,c,d*/;
    const int n_steps_ = 2;
    opt_var_.AddVariableSet("SplineCoff", Eigen::VectorXd(n_coeff_));
    opt_var_.AddVariableSet("Footholds", Eigen::VectorXd(n_steps_*2));

    constraints_ = std::make_shared<ConstraintContainer>(opt_var_);

    int n_spline_coeff = opt_var_.GetVariables(VariableNames::kSplineCoeff).rows();
    c_zeros_ = CostConstraintFactory::CreateAccConstraint(Eigen::Vector2d::Zero(),splines_);
    c_ones_  = CostConstraintFactory::CreateAccConstraint(Eigen::Vector2d::Ones(),splines_);

    constraints_->AddConstraint(c_zeros_);
    constraints_->AddConstraint(c_ones_);
  }

  OptimizationVariables opt_var_;
  ConstraintPtr c_zeros_, c_ones_;
  ConstraintContainerPtr constraints_;
  ComSpline4 splines_;
};

TEST_F(ConstraintContainerTest, EvaluateConstraintsInitialAcc)
{
  Eigen::VectorXd g = constraints_->EvaluateConstraints();
  std::cout << "g: " << g.transpose() << std::endl;

  EXPECT_EQ(4, g.rows()); // two constraints in x and one in y

  // remember: constraints stored in std::map, so no ordering (e.g. not first added first out)
  EXPECT_EQ(c_zeros_->EvaluateConstraint(), g.tail<2>()); // two constraints in x and one in y
  EXPECT_EQ(c_ones_->EvaluateConstraint() , g.head<2>()); // two constraints in x and one in y
}

TEST_F(ConstraintContainerTest, GetBoundsInitialAcc)
{
  ConstraintContainer::VecBound bounds = constraints_->GetBounds();

  EXPECT_EQ(4, bounds.size()); // two constraints in x and one in y

  EXPECT_EQ(0.0, bounds.at(0).lower_); EXPECT_EQ(0.0, bounds.at(0).upper_); // x direction
  EXPECT_EQ(0.0, bounds.at(1).lower_); EXPECT_EQ(0.0, bounds.at(1).upper_); // y direction
  EXPECT_EQ(1.0, bounds.at(2).lower_); EXPECT_EQ(1.0, bounds.at(2).upper_); // x direction
  EXPECT_EQ(1.0, bounds.at(3).lower_); EXPECT_EQ(1.0, bounds.at(3).upper_); // y direction
}

} /* namespace zmp */
} /* namespace xpp */
