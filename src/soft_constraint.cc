/**
 @file    cost_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Defines the class SoftConstraint
 */

#include <xpp/soft_constraint.h>

#include <Eigen/Sparse>

#include <xpp/bound.h>

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

void
SoftConstraint::Update ()
{
  constraint_->UpdateConstraintValues();
  constraint_->UpdateJacobians();
}

double
SoftConstraint::EvaluateCost () const
{
  VectorXd g = constraint_->GetConstraintValues();
  return 0.5*(g-b_).transpose()*weights_.asDiagonal()*(g-b_);
}

SoftConstraint::VectorXd
SoftConstraint::EvaluateGradientWrt (std::string var_set)
{
  VectorXd g = constraint_->GetConstraintValues();
  Constraint::Jacobian jac = constraint_->GetJacobianWithRespectTo(var_set);

  VectorXd grad;
  if (jac.rows() != 0)
    grad = jac.transpose()*weights_.asDiagonal()*(g-b_);

  return grad;
}

} /* namespace opt */
} /* namespace xpp */
