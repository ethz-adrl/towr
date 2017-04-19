/**
 @file    nlp.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <xpp/nlp.h>

#include <algorithm>
#include <Eigen/Sparse>

namespace xpp {
namespace opt {

NLP::NLP ()
{
}

NLP::~NLP ()
{
  // TODO Auto-generated destructor stub
}

void
NLP::Init (OptimizationVariablesPtr& opt_variables)
{
  opt_variables_ = opt_variables;
}

int
NLP::GetNumberOfOptimizationVariables () const
{
  return opt_variables_->GetOptimizationVariableCount();
}

VecBound
NLP::GetBoundsOnOptimizationVariables () const
{
  return opt_variables_->GetOptimizationVariableBounds();
}

NLP::VectorXd
NLP::GetStartingValues () const
{
  return opt_variables_->GetOptimizationVariables();
}

void
NLP::SetVariables (const Number* x)
{
  opt_variables_->SetAllVariables(ConvertToEigen(x));
}

double
NLP::EvaluateCostFunction (const Number* x)
{
  SetVariables(x);
  VectorXd g = costs_.GetWeightedCost();
  assert(g.rows() == 1);
  return g(0);
}

NLP::VectorXd
NLP::EvaluateCostFunctionGradient (const Number* x)
{
  SetVariables(x);
  Jacobian jac = costs_.GetWeightedJacobian();
  assert(jac.rows() == 1);
  return jac.row(0).transpose();
}

VecBound
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
NLP::EvaluateConstraints (const Number* x)
{
  SetVariables(x);
  return constraints_->GetConstraintValues();
}

bool
NLP::HasCostTerms () const
{
  return !costs_.IsEmpty();
}

void
NLP::EvalNonzerosOfJacobian (const Number* x, Number* values)
{
  SetVariables(x);
  Jacobian jac = GetJacobianOfConstraints();

  jac.makeCompressed(); // so the valuePtr() is dense and accurate
  std::copy(jac.valuePtr(), jac.valuePtr() + jac.nonZeros(), values);
}

NLP::Jacobian
NLP::GetJacobianOfConstraints () const
{
  return constraints_->GetConstraintJacobian();
}

void
NLP::AddCost (CostPtr cost, double weight)
{
  costs_.AddCost(cost, weight);
}

void
NLP::AddConstraint (ConstraintPtrU c)
{
  constraints_ = std::move(c);
}

NLP::VectorXd
NLP::ConvertToEigen(const Number* x) const
{
  return Eigen::Map<const VectorXd>(x,GetNumberOfOptimizationVariables());
}

void
NLP::Reset ()
{
  costs_.ClearCosts();
}

} /* namespace opt */
} /* namespace xpp */

