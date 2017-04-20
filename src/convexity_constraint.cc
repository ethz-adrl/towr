/**
 @file    convexity_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/convexity_constraint.h>

#include <algorithm>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/endeffectors.h>

#include <xpp/bound.h>
#include <xpp/opt/variables/endeffector_load.h>

namespace xpp {
namespace opt {

ConvexityConstraint::ConvexityConstraint (const OptVarsPtr& opt_vars)
{
  name_ = "ConvexityConstraint";
  ee_load_ = std::dynamic_pointer_cast<EndeffectorLoad>(opt_vars->GetSet("endeffector_load"));

  int m = ee_load_->GetNumberOfSegments();
  SetDimensions(opt_vars, m);
}

ConvexityConstraint::~ConvexityConstraint ()
{
}

ConvexityConstraint::VectorXd
ConvexityConstraint::GetValues () const
{
  int m = GetRows();
  VectorXd g(m);

  for (int k=0; k<m; ++k) {

    double sum_k = 0.0;

    for (double lambda : ee_load_->GetLoadValuesIdx(k).ToImpl())
      sum_k += lambda;

    g(k) = sum_k; // sum equal to 1
  }

  return g;
}

VecBound
ConvexityConstraint::GetBounds () const
{
  VecBound b(GetRows());
  std::fill(b.begin(), b.end(), Bound(1.0, 1.0));
  return b;
}

void
ConvexityConstraint::FillJacobianWithRespectTo (std::string var_set,
                                               Jacobian& jac) const
{
  if (var_set == ee_load_->GetId()) {
    for (int k=0; k<ee_load_->GetNumberOfSegments(); ++k) {
      for (auto ee : ee_load_->GetLoadValuesIdx(k).GetEEsOrdered()) {
        int idx = ee_load_->IndexDiscrete(k,ee);
        jac.insert(k, idx) = 1.0;
      }
    }
  }
}

} /* namespace opt */
} /* namespace xpp */
