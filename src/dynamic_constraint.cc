/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Defines the DynamicConstraint class
 */

#include <xpp/opt/dynamic_constraint.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint ()
{
  // TODO Auto-generated constructor stub
}

DynamicConstraint::~DynamicConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
DynamicConstraint::UpdateVariables (const OptimizationVariables*)
{
}

DynamicConstraint::VectorXd
DynamicConstraint::EvaluateConstraint () const
{
}

DynamicConstraint::VecBound
DynamicConstraint::GetBounds () const
{
}

DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
}

} /* namespace opt */
} /* namespace xpp */
