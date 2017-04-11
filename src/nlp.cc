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
  opt_variables_ = /*std::move*/(opt_variables);
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
  constraints_.UpdateConstraints();
  costs_.UpdateCosts();
}

double
NLP::EvaluateCostFunction (const Number* x)
{
  SetVariables(x);
  return costs_.EvaluateTotalCost();
}

NLP::VectorXd
NLP::EvaluateCostFunctionGradient (const Number* x)
{
  SetVariables(x);

  // analytical (if implemented in costs)
  VectorXd grad = costs_.EvaluateGradient();

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

VecBound
NLP::GetBoundsOnConstraints () const
{
  return constraints_.GetBounds();
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
  return constraints_.EvaluateConstraints();
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
  JacobianPtr jac = GetJacobianOfConstraints();

  jac->makeCompressed(); // so the valuePtr() is dense and accurate
  std::copy(jac->valuePtr(), jac->valuePtr() + jac->nonZeros(), values);
}

NLP::JacobianPtr
NLP::GetJacobianOfConstraints () const
{
  return constraints_.GetJacobian();
}

void
NLP::PrintStatusOfConstraints (double tol) const
{
  return constraints_.PrintStatus(tol);
}

void
NLP::AddCost (CostPtr cost, double weight)
{
  costs_.AddCost(cost, weight);
}

void
NLP::AddConstraint (ConstraitPtrVec constraints)
{
  constraints_.AddConstraint(constraints);
}

NLP::VectorXd
NLP::ConvertToEigen(const Number* x) const
{
  return Eigen::Map<const VectorXd>(x,GetNumberOfOptimizationVariables());
}

} /* namespace opt */
} /* namespace xpp */
