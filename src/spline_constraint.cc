/**
 @file    spline_junction_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/constraints/spline_constraint.h>

namespace xpp {
namespace opt {

SplineConstraint::SplineConstraint (const OptVarsPtr& opt_vars,
                                    const std::string& spline_id,
                                    const DerivativeVec& derivatives
                                    )
{
  spline_          = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(spline_id));
//  contact_timings_ = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(spline_id));

  derivatives_ = derivatives;
  n_dim_ = spline_->GetNDim();

  AddOptimizationVariables(opt_vars);
}

SplineConstraint::~SplineConstraint ()
{
}





SplineStateConstraint::SplineStateConstraint (const OptVarsPtr& opt_vars,
                                              const std::string& spline_id,
                                              double t,
                                              const StateLinXd& state,
                                              const DerivativeVec& derivatives)
    :SplineConstraint(opt_vars, spline_id, derivatives)
{
  SetName("New SplineStateConstraint-" + spline_id);

  t_ = t;
  state_desired_ = state;
  int n_constraints = derivatives.size()*n_dim_;
  SetRows(n_constraints);
}

SplineStateConstraint::~SplineStateConstraint ()
{
}

VectorXd
SplineStateConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());

  StateLinXd state_spline = spline_->GetPoint(t_);

  int row = 0; // constraint count
  for (auto dxdt :  derivatives_) {

    g.middleRows(row,n_dim_) = state_spline.GetByIndex(dxdt);
    row += n_dim_;
  }

  return g;
}

void
SplineStateConstraint::FillJacobianWithRespectTo (std::string var_set,
                                                  Jacobian& jac) const
{
  if (var_set == spline_->GetName()) {

    int row = 0;
    for (auto dxdt :  derivatives_) {
      jac.middleRows(row,n_dim_) =  spline_->GetJacobian(t_, dxdt);
      row += n_dim_;
    }
  }
}

VecBound
SplineStateConstraint::GetBounds () const
{
  VecBound bounds;

  for (auto dxdt :  derivatives_) {
    VectorXd state = state_desired_.GetByIndex(dxdt);
    for (int dim=0; dim<n_dim_; ++dim) {
      bounds.push_back(Bound(state(dim), state(dim)));
    }
  }

  return bounds;
}










SplineJunctionConstraint::SplineJunctionConstraint (const OptVarsPtr& opt_vars,
                                    const std::string& spline_id,
                                    const DerivativeVec& derivatives
                                    )
    :SplineConstraint(opt_vars, spline_id, derivatives)
{
  SetName("New SplineJunctionConstraint-" + spline_id);

  n_junctions_ = spline_->GetPolynomials().size()-1; // because one less junction than poly's.
  int n_constraints = derivatives_.size() * n_junctions_ * n_dim_;
  SetRows(n_constraints);
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
    double T = spline_->GetDurationOfPoly(id);

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
      double T = spline_->GetDurationOfPoly(id);

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


