/**
 @file    a_linear_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/a_linear_constraint.h>

namespace xpp {
namespace opt {

ALinearConstraint::ALinearConstraint ()
{
}

void
ALinearConstraint::Init (const MatVec& linear_equation, const std::string& name)
{
  linear_equation_ = linear_equation;
  name_ = name;
}

ALinearConstraint::VectorXd
ALinearConstraint::EvaluateConstraint () const
{
  return linear_equation_.M*x_; // linear part respected in bounds
}

VecBound
LinearEqualityConstraint::GetBounds () const
{
  VecBound bounds;
  for (int i=0; i<linear_equation_.v.rows(); ++i) {
    Bound bound(-linear_equation_.v[i],-linear_equation_.v[i]);
    bounds.push_back(bound);
  }
  return bounds;
}

VecBound
LinearInequalityConstraint::GetBounds () const
{
  VecBound bounds;
  for (int i=0; i<linear_equation_.v.rows(); ++i) {
    Bound bound(-linear_equation_.v[i], kNoBound_.upper_);
    bounds.push_back(bound);
  }
  return bounds;
}

} /* namespace opt */
} /* namespace xpp */

