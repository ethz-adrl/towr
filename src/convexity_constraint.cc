/**
 @file    convexity_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/convexity_constraint.h>
#include <xpp/opt/endeffector_load.h>

namespace xpp {
namespace opt {

ConvexityConstraint::ConvexityConstraint (const OptVarsPtr& opt_vars)
{
  name_ = "Convexity";
  ee_load_ = std::dynamic_pointer_cast<EndeffectorLoad>(opt_vars->GetSet("endeffector_load"));

  int m = ee_load_->GetNumberOfSegments();
  SetDimensions(opt_vars->GetOptVarsVec(), m);

  Jacobian& jac = GetJacobianRefWithRespectTo(ee_load_->GetId());

  for (int k=0; k<m; ++k) {
    for (auto ee : ee_load_->GetLoadValuesIdx(k).GetEEsOrdered()) {
      int idx = ee_load_->IndexDiscrete(k,ee);
      jac.insert(k, idx) = 1.0;
    }
  }
}

ConvexityConstraint::~ConvexityConstraint ()
{
}

void
ConvexityConstraint::UpdateConstraintValues ()
{
  for (int k=0; k<GetNumberOfConstraints(); ++k) {

    double sum_k = 0.0;

    for (double lambda : ee_load_->GetLoadValuesIdx(k).ToImpl())
      sum_k += lambda;

    g_(k) = sum_k; // sum equal to 1
  }
}

void
ConvexityConstraint::UpdateBounds ()
{
  std::fill(bounds_.begin(), bounds_.end(), Bound(1.0, 1.0));
}

} /* namespace opt */
} /* namespace xpp */
