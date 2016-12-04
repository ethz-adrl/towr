/**
 @file    convexity_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/convexity_constraint.h>
#include <xpp/opt/motion_structure.h>

namespace xpp {
namespace opt {

ConvexityConstraint::ConvexityConstraint ()
{
  // TODO Auto-generated constructor stub
}

ConvexityConstraint::~ConvexityConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
ConvexityConstraint::Init (const MotionStructure& motion_structure)
{
}

void
ConvexityConstraint::UpdateVariables (const OptimizationVariables*)
{
}

ConvexityConstraint::VectorXd
ConvexityConstraint::EvaluateConstraint () const
{
}

ConvexityConstraint::VecBound
ConvexityConstraint::GetBounds () const
{
}

ConvexityConstraint::Jacobian
ConvexityConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
}

} /* namespace opt */
} /* namespace xpp */
