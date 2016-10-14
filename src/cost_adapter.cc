/**
 @file    cost_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Defines the class CostAdapter
 */

#include <xpp/opt/cost_adapter.h>

namespace xpp {
namespace opt {

CostAdapter::~CostAdapter ()
{
  // TODO Auto-generated destructor stub
}

CostAdapter::CostAdapter (const ConstraintPtr& constraint)
{
  constraint_ = constraint;
  int n_constraints = constraint->GetNumberOfConstraints();

  // treat all constraints equally by default
  weights_.resize(n_constraints);
  weights_.setOnes();
}

void
CostAdapter::SetWeights (const VectorXd& weights)
{
  weights_ = weights;
}

void
CostAdapter::UpdateVariables (const OptimizationVariables* x)
{
  constraint_->UpdateVariables(x);
}

double
CostAdapter::EvaluateCost () const
{
  VectorXd g = constraint_->EvaluateConstraint();
  return weights_.transpose()*g;
}

CostAdapter::VectorXd
CostAdapter::EvaluateGradientWrt (std::string var_set)
{
  AConstraint::Jacobian jac = constraint_->GetJacobianWithRespectTo(var_set);
  return jac.transpose()*weights_;
}

} /* namespace opt */
} /* namespace xpp */
