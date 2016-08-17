/**
 @file    nlp_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <xpp/zmp/nlp.h>
#include <xpp/zmp/ipopt_adapter.h>
#include <IpIpoptApplication.hpp>

#include <gtest/gtest.h>

namespace xpp {
namespace zmp {

typedef Eigen::VectorXd VectorXd;
typedef ConstraintContainer::ConstraintPtr ConstraintPtr;
static const std::string set1 = "set1_variables";
static const std::string set2 = "set2_variables";


class FooConstraint : public AConstraint {
public:
  void UpdateVariables(const OptimizationVariables* opt_var)
  {
    x_set1 = opt_var->GetVariables(set1);
    x_set2 = opt_var->GetVariables(set2);
  }

  VectorXd EvaluateConstraint () const
  {
    // two constraints, where the second also depends on the second variable set
    VectorXd g(m);
    g[0] = 1*x_set1[0] + 2*x_set1[1] - 4.0;
    g[1] = x_set1[0]*x_set1[0] + 4*x_set2[0];
    return g;
  }

  Jacobian GetJacobianWithRespectTo (std::string var_set) const
  {
    Jacobian jac; // empy default

    if (var_set == set1) {
      jac = Jacobian(m,x_set1.rows()); // two constraint, two optimization variables;
      jac.insert(0,0) = 1;
      jac.insert(0,1) = 2;
      jac.insert(1,0) = 2*x_set1[0];
    }

    if (var_set == set2) {
      jac = Jacobian(m,x_set2.rows()); // two constraint, one optimization variables;
      jac.insert(1,0) = 4;
    }

    return jac;
  }

  VecBound GetBounds () const {
    VecBound bounds;
    bounds.push_back(AConstraint::kEqualityBound_);
    bounds.push_back(AConstraint::kEqualityBound_);
    return bounds;
  }

private:
  VectorXd x_set1;
  VectorXd x_set2;
  const int m = 2; // number of constraints

};

class NlpTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    opt_variables_     = std::make_shared<OptimizationVariables>();
    auto costs         = std::make_shared<CostContainer>(*opt_variables_);
    auto constraints   = std::make_shared<ConstraintContainer>(*opt_variables_);

    opt_variables_->AddVariableSet(set1, VectorXd(2));
    opt_variables_->AddVariableSet(set2, VectorXd(1));

    auto foo_constraint = std::make_shared<FooConstraint>();
    constraints->AddConstraint(foo_constraint, "foo_constraint");

    nlp_.Init(opt_variables_, costs, constraints);
  }

  std::shared_ptr<OptimizationVariables> opt_variables_;
  NLP nlp_;
};

TEST_F(NlpTest, ApproximatingJacobian)
{
  auto nlp_ipopt = new IpoptAdapter(nlp_);
  Ipopt::IpoptApplication solver;
  solver.Options()->SetStringValue("jacobian_approximation", "finite-difference-values");
  solver.Options()->SetStringValue("hessian_approximation", "limited-memory");
  solver.OptimizeTNLP(nlp_ipopt);

  double tol = 1e-1;
  EXPECT_NEAR(0.749747, opt_variables_->GetOptimizationVariables()[0], tol);
  EXPECT_NEAR( 1.62513, opt_variables_->GetOptimizationVariables()[1], tol);
  EXPECT_NEAR(-0.14053, opt_variables_->GetOptimizationVariables()[2], tol);
  std::cout << opt_variables_->GetOptimizationVariables().transpose() << std::endl;
}

TEST_F(NlpTest, AnalyticalJacobian)
{
  auto nlp_ipopt = new IpoptAdapter(nlp_);
  Ipopt::IpoptApplication solver;
  solver.Options()->SetStringValue("hessian_approximation", "limited-memory");
  solver.OptimizeTNLP(nlp_ipopt);

  double tol = 1e-1;
  EXPECT_NEAR(0.749747, opt_variables_->GetOptimizationVariables()[0], tol);
  EXPECT_NEAR( 1.62513, opt_variables_->GetOptimizationVariables()[1], tol);
  EXPECT_NEAR(-0.14053, opt_variables_->GetOptimizationVariables()[2], tol);
  std::cout << opt_variables_->GetOptimizationVariables().transpose() << std::endl;
}

} /* namespace zmp */
} /* namespace xpp */
