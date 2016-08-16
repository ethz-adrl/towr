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
static const std::string set1 = "coeff";
//static const std::string set2 = "feet";


class FooConstraint : public AConstraint {
public:
  void UpdateVariables(const OptimizationVariables* opt_var) {
    x = opt_var->GetVariables(set1);
  }

  VectorXd EvaluateConstraint () const
  {
    VectorXd g(1);
    g[0] = x[0] + 2*x[1] - 4.0;
    return g;
  }

  Jacobian GetJacobianWithRespectTo (std::string var_set) const
  {
    Jacobian jac; // empy default

    if (var_set == set1) {
      jac = Jacobian(1,2); // one constraint, two optimization variables;
      jac(0,0) = 1;
      jac(0,1) = 2;
    }

    return jac;
  }

  VecBound GetBounds () const {
    VecBound bounds;
    bounds.push_back(AConstraint::kEqualityBound_);
    return bounds;
  }

private:
  VectorXd x;

};

class NlpTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    opt_variables_     = std::make_shared<OptimizationVariables>();
    auto costs         = std::make_shared<CostContainer>(*opt_variables_);
    auto constraints   = std::make_shared<ConstraintContainer>(*opt_variables_);

    opt_variables_->AddVariableSet(set1, VectorXd(2));
//    opt_variables_->AddVariableSet(set2, VectorXd(3));

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

  std::cout << opt_variables_->GetOptimizationVariables().transpose() << std::endl;
}

TEST_F(NlpTest, AnalyticalJacobian)
{
  auto nlp_ipopt = new IpoptAdapter(nlp_);
  Ipopt::IpoptApplication solver;
  solver.Options()->SetStringValue("hessian_approximation", "limited-memory");
  solver.OptimizeTNLP(nlp_ipopt);

  std::cout << opt_variables_->GetOptimizationVariables().transpose() << std::endl;
}

} /* namespace zmp */
} /* namespace xpp */
