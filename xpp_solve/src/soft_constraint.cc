/**
 @file    cost_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Defines the class SoftConstraint
 */

#include <xpp_solve/soft_constraint.h>


namespace opt {

SoftConstraint::SoftConstraint (const ConstraintPtr& constraint, double weight)
    :Component(1, "SoftConstraint-" + constraint->GetName())
{
  constraint_ = constraint;
  int n_constraints = constraint_->GetRows();

  // average value of each upper and lower bound
  b_ = VectorXd(n_constraints);
  int i=0;
  for (auto b : constraint_->GetBounds()) {
    b_(i++) = (b.upper_ + b.lower_)/2.;
  }

  // treat all constraints equally by default
  weights_.resize(n_constraints);
  weights_.setOnes();

  weight_ = weight;
}

SoftConstraint::~SoftConstraint ()
{
}

SoftConstraint::VectorXd
SoftConstraint::GetValues () const
{
  VectorXd g = constraint_->GetValues();
  VectorXd cost = 0.5*(g-b_).transpose()*weights_.asDiagonal()*(g-b_);
  return weight_ * cost;
}

SoftConstraint::Jacobian
SoftConstraint::GetJacobian () const
{
  VectorXd g   = constraint_->GetValues();
  Jacobian jac = constraint_->GetJacobian();
  VectorXd grad = weight_*jac.transpose()*weights_.asDiagonal()*(g-b_);
  return grad.transpose().sparseView();
}

} /* namespace opt */
