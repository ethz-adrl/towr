/**
 @file    cost_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Defines the class CostAdapter
 */

#include <xpp/opt/cost_adapter.h>
#include <iostream>

namespace xpp {
namespace opt {

CostAdapter::~CostAdapter ()
{
  // TODO Auto-generated destructor stub
}

CostAdapter::CostAdapter (const ConstraintPtr& constraint)
{
  // zmp_ here the ROM cost is probably not correctly initialized yet...
  constraint_ = constraint;
  int n_constraints = constraint_->GetNumberOfConstraints();

  // zmp_ is constant, move to constructor
  // average value of each upper and lower bound
  b_ = VectorXd(n_constraints);
  int i=0;
  for (auto b : constraint_->GetBounds()) {
    b_(i++) = (b.upper_ + b.lower_)/2.;
  }

  // treat all constraints equally by default
  weights_.resize(n_constraints);
  weights_.setOnes();
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
  return 0.5*(g-b_).transpose()*weights_.asDiagonal()*(g-b_);
}

CostAdapter::VectorXd
CostAdapter::EvaluateGradientWrt (std::string var_set)
{
  VectorXd g = constraint_->EvaluateConstraint();
  AConstraint::Jacobian jac = constraint_->GetJacobianWithRespectTo(var_set);

  VectorXd grad;
  if (jac.rows() != 0)
    grad = jac.transpose()*weights_.asDiagonal()*(g-b_);

  return grad;
}

} /* namespace opt */
} /* namespace xpp */
