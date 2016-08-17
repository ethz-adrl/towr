/**
 @file    nlp.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <xpp/zmp/nlp.h>

namespace xpp {
namespace zmp {

NLP::NLP ()
    :cost_derivative_(1*std::numeric_limits<double>::epsilon())
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
  opt_variables_->SetVariables(ConvertToEigen(x));
  Eigen::MatrixXd jacobian(1, GetNumberOfOptimizationVariables());
  cost_derivative_.df(opt_variables_->GetOptimizationVariables(), jacobian);
  return jacobian.transpose();
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
  auto jac = constraints_->GetJacobian();

  jac->makeCompressed(); // so the valuePtr() is dense and accurate
  std::copy(jac->valuePtr(), jac->valuePtr() + jac->nonZeros(), values);
}

NLP::Jacobian
NLP::GetJacobianOfConstraints () const
{
  return *constraints_->GetJacobian();
}

NLP::VectorXd
NLP::ConvertToEigen(const Number* x) const
{
  return Eigen::Map<const VectorXd>(x,GetNumberOfOptimizationVariables());
}

} /* namespace zmp */
} /* namespace xpp */


