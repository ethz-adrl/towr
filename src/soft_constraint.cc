/**
 @file    cost_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Defines the class SoftConstraint
 */

#include <xpp/soft_constraint.h>

#include <Eigen/Sparse>

#include <xpp/bound.h>
#include <xpp/opt/constraints/constraint.h>

namespace xpp {
namespace opt {

SoftConstraint::SoftConstraint (const OptVarsPtr& opt_vars_container,
                                const ConstraintPtr& constraint)
    :Cost(opt_vars_container)
{
  constraint_ = constraint;
  int n_constraints = constraint_->GetNumberOfConstraints();

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

SoftConstraint::~SoftConstraint ()
{
}

double
SoftConstraint::GetCost () const
{
  VectorXd g = constraint_->GetConstraintValues();
  return 0.5*(g-b_).transpose()*weights_.asDiagonal()*(g-b_);
}

SoftConstraint::Jacobian
SoftConstraint::GetJacobian () const
{
  VectorXd g   = constraint_->GetConstraintValues();
  Jacobian jac = constraint_->GetConstraintJacobian();
  VectorXd grad = jac.transpose()*weights_.asDiagonal()*(g-b_);
  return grad.transpose().sparseView();
}

} /* namespace opt */
} /* namespace xpp */
