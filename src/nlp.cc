/**
 @file    nlp.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <xpp/opt/nlp.h>

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
NLP::Init (const OptimizationVariablesPtr& opt_variables)
{
  opt_variables_ = opt_variables;
}

int
NLP::GetNumberOfOptimizationVariables () const
{
  return opt_variables_->GetRows();
}

VecBound
NLP::GetBoundsOnOptimizationVariables () const
{
  return opt_variables_->GetBounds();
}

VectorXd
NLP::GetStartingValues () const
{
  return opt_variables_->GetValues();
}

void
NLP::SetVariables (const Number* x)
{
  opt_variables_->SetValues(ConvertToEigen(x));
}

double
NLP::EvaluateCostFunction (const Number* x)
{
  VectorXd g = VectorXd::Zero(1);
  if (HasCostTerms()) {
    SetVariables(x);
    g = costs_->GetValues();
  }
  return g(0);
}

VectorXd
NLP::EvaluateCostFunctionGradient (const Number* x)
{
  Jacobian jac = Jacobian(1,GetNumberOfOptimizationVariables());
  if (HasCostTerms()) {
    SetVariables(x);
    jac = costs_->GetJacobian();
  }

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
  return constraints_->GetRows();
//  return GetBoundsOnConstraints().size();
}

VectorXd
NLP::EvaluateConstraints (const Number* x)
{
  SetVariables(x);
  return constraints_->GetValues();
}

bool
NLP::HasCostTerms () const
{
  return costs_->GetRows()>0;
}

void
NLP::EvalNonzerosOfJacobian (const Number* x, Number* values)
{
  SetVariables(x);
  Jacobian jac = GetJacobianOfConstraints();

  jac.makeCompressed(); // so the valuePtr() is dense and accurate
  std::copy(jac.valuePtr(), jac.valuePtr() + jac.nonZeros(), values);
}

Jacobian
NLP::GetJacobianOfConstraints () const
{
  return constraints_->GetJacobian();
}

void
NLP::AddCost (ConstraintPtrU cost)
{
  costs_ = std::move(cost);
}

void
NLP::AddConstraint (ConstraintPtrU constraint)
{
  constraints_ = std::move(constraint);
}

VectorXd
NLP::ConvertToEigen(const Number* x) const
{
  return Eigen::Map<const VectorXd>(x,GetNumberOfOptimizationVariables());
}

} /* namespace opt */
} /* namespace xpp */

