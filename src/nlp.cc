/**
 @file    nlp.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <xpp/opt/nlp.h>

namespace xpp {
namespace opt {

NLP::NLP ()
    :cost_derivative_(std::numeric_limits<double>::epsilon())
{
}

NLP::~NLP ()
{
  // TODO Auto-generated destructor stub
}

void
NLP::Init (OptimizationVariablesPtr& opt_variables,
           CostContainerPtr& costs,
           ConstraintContainerPtr& constraints)
{
  opt_variables_ = /*std::move*/(opt_variables);
  costs_         = /*std::move*/(costs);
  constraints_   = /*std::move*/(constraints);

  cost_derivative_.AddCosts(*opt_variables_, *costs_);
}

int
NLP::GetNumberOfOptimizationVariables () const
{
  return opt_variables_->GetOptimizationVariableCount();
}

NLP::BoundVec
NLP::GetBoundsOnOptimizationVariables () const
{
  return opt_variables_->GetOptimizationVariableBounds();
}

NLP::VectorXd
NLP::GetStartingValues () const
{
  return opt_variables_->GetOptimizationVariables();
}

double
NLP::EvaluateCostFunction (const Number* x) const
{
  opt_variables_->SetVariables(ConvertToEigen(x));
  return costs_->EvaluateTotalCost();
}

NLP::VectorXd
NLP::EvaluateCostFunctionGradient (const Number* x) const
{
  // motion_ref use matrix acceleration jacobian for this
  opt_variables_->SetVariables(ConvertToEigen(x));

  // analytical (if implemented in costs)
  VectorXd grad = costs_->EvaluateGradient();

  // refactor move numerical calculation of cost to cost_container class,
  // so some can be implemented analytical, others using numerical differentiaton.

//  // motion_ref don't forget bout this
//  // To just test for feasability
//  VectorXd grad(opt_variables_->GetOptimizationVariableCount());
//  grad.setZero();

//  // numerical differentiation
//  Eigen::MatrixXd jacobian(1, GetNumberOfOptimizationVariables());
//  cost_derivative_.df(opt_variables_->GetOptimizationVariables(), jacobian);
//  VectorXd grad = jacobian.transpose();

  return grad;
}

NLP::BoundVec
NLP::GetBoundsOnConstraints () const
{
  return constraints_->GetBounds();
}

int
NLP::GetNumberOfConstraints () const
{
  return GetBoundsOnConstraints().size();
}

NLP::VectorXd
NLP::EvaluateConstraints (const Number* x) const
{
  opt_variables_->SetVariables(ConvertToEigen(x));
  return constraints_->EvaluateConstraints();
}

void
NLP::SetVariables (const Number* x)
{
  opt_variables_->SetVariables(ConvertToEigen(x));
}

bool
NLP::HasCostTerms () const
{
  return !costs_->IsEmpty();
}

void
NLP::EvalNonzerosOfJacobian (const Number* x, Number* values) const
{
  opt_variables_->SetVariables(ConvertToEigen(x));
  JacobianPtr jac = GetJacobianOfConstraints();

  jac->makeCompressed(); // so the valuePtr() is dense and accurate
  std::copy(jac->valuePtr(), jac->valuePtr() + jac->nonZeros(), values);
}

NLP::JacobianPtr
NLP::GetJacobianOfConstraints () const
{
  // use full default jacobian if not estimated, to make sure all the
  // elements are estimated by numerical differences.
  // refactor this is something I will for sure forget, remove
  bool jacobians_defined = true;

  if (jacobians_defined)
    return constraints_->GetJacobian();
  else {
    int n = GetNumberOfOptimizationVariables();
    int m = GetNumberOfConstraints();
    Eigen::MatrixXd jac_default(m,n);
    jac_default.setOnes(); // to make sure every element is respected
    JacobianPtr ptr = std::make_shared<Jacobian>(jac_default.sparseView());
    return ptr;
  }
}

NLP::VectorXd
NLP::ConvertToEigen(const Number* x) const
{
  return Eigen::Map<const VectorXd>(x,GetNumberOfOptimizationVariables());
}

} /* namespace zmp */
} /* namespace xpp */


