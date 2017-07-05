/**
 @file    spline_junction_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/constraints/spline_junction_constraint.h>

namespace xpp {
namespace opt {

SplineJunctionConstraint::SplineJunctionConstraint (const OptVarsPtr& opt_vars,
                                                    const std::string& spline_id,
                                                    const DerivativeVec& derivatives
                                                    )
{
  SetName("New SplineJunctionConstraint-" + spline_id);

  spline_          = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(spline_id));
//  contact_timings_ = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(spline_id));

  derivatives_ = derivatives;
  n_dim_ = spline_->GetNDim();
  n_junctions_ = spline_->GetPolynomials().size()-1; // because one less junction than poly's.
  int n_constraints = derivatives_.size() * n_junctions_ * n_dim_;

  SetRows(n_constraints);
  AddOptimizationVariables(opt_vars);
}

SplineJunctionConstraint::~SplineJunctionConstraint ()
{
}

VectorXd
SplineJunctionConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());

  int row = 0;

  for (int id = 0; id < n_junctions_; ++id) {
    double T = spline_->GetPolynomials().at(id)->GetDuration();

    auto p0 = spline_->GetPoint(id, T);
    auto p1 = spline_->GetPoint(id+1, 0.0);

    for (auto dxdt :  derivatives_) {
      g.middleRows(row,n_dim_) = p0.GetByIndex(dxdt) - p1.GetByIndex(dxdt);
      row += n_dim_;
    }
  }

  return g;
}

VecBound
SplineJunctionConstraint::GetBounds () const
{
  VecBound bounds(GetRows());
  std::fill(bounds.begin(), bounds.end(), kEqualityBound_);

  return bounds;
}

void
SplineJunctionConstraint::FillJacobianWithRespectTo (std::string var_set,
                                                     Jacobian& jac) const
{
  if (var_set == spline_->GetName()) {

    int row = 0;
    for (int id = 0; id < n_junctions_; ++id) {
      double T = spline_->GetPolynomials().at(id)->GetDuration();

      for (auto dxdt :  derivatives_) {

        auto jac_0 = spline_->GetJacobian(id, T, dxdt);
        auto jac_1 = spline_->GetJacobian(id+1, 0.0, dxdt);

        jac.middleRows(row,n_dim_) = jac_0 - jac_1;
        row += n_dim_;
      }
    }
  }

}

} /* namespace opt */
} /* namespace xpp */
