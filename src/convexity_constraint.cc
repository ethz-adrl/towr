/**
 @file    convexity_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/convexity_constraint.h>

namespace xpp {
namespace opt {

ConvexityConstraint::ConvexityConstraint (const LoadPtr& ee_load)
{
  name_ = "Convexity";
  ee_load_ = ee_load;

  int m = ee_load->GetNumberOfSegments();
  SetDependentVariables({ee_load}, m);

  Jacobian& jac = GetJacobianRefWithRespectTo(ee_load->GetId());

  for (int k=0; k<m; ++k) {
    for (auto ee : ee_load->GetLoadValuesIdx(k).GetEEsOrdered()) {
      int idx = ee_load->IndexDiscrete(k,ee);
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
