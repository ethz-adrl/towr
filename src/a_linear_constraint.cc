/**
 @file    a_linear_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/zmp/a_linear_constraint.h>

namespace xpp {
namespace zmp {

ALinearConstraint::ALinearConstraint (OptimizationVariables& subject)
{
  subject_ = &subject;
  subject_->RegisterObserver(this);
}

void
ALinearConstraint::Init (const MatVec& linear_equation)
{
  linear_equation_ = linear_equation;
  Update();
}

void
ALinearConstraint::Update ()
{
  x_coeff_ = subject_->GetSplineCoefficients();
}

ALinearConstraint::VectorXd
ALinearConstraint::EvaluateConstraint () const
{
  return linear_equation_.M*x_coeff_ + linear_equation_.v;
}

ALinearConstraint::VecBound
ALinearConstraint::GetBounds () const
{
  VecBound bounds;
  for (int i=0; i<linear_equation_.v.rows(); ++i)
    bounds.push_back(bound_);

  return bounds;
}

LinearEqualityConstraint::LinearEqualityConstraint (
    OptimizationVariables& subject)
  :ALinearConstraint(subject)
{
  bound_ = kEqualityBound_;
}

LinearInequalityConstraint::LinearInequalityConstraint (
    OptimizationVariables& subject)
  :ALinearConstraint(subject)
{
  bound_ = kInequalityBoundPositive_;
}

} /* namespace zmp */
} /* namespace xpp */

