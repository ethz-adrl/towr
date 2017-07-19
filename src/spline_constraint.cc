/**
 @file    spline_junction_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/constraints/spline_constraint.h>

namespace xpp {
namespace opt {


SplineStateConstraint::SplineStateConstraint (const OptVarsPtr& opt_vars,
                                              PolyPtr active_poly,
                                              double t_local,
                                              const StateLinXd& state,
                                              const DerivativeVec& derivatives)
{
  SetName("SplineStateConstraint-" + active_poly->GetName());

  active_poly_ = active_poly;
  t_local_     = t_local;

  state_desired_ = state;
  derivatives_   = derivatives;
  n_dim_         = state.kNumDim;

  int n_constraints = derivatives.size()*n_dim_;
  AddOptimizationVariables(opt_vars);
  SetRows(n_constraints);
}

SplineStateConstraint::~SplineStateConstraint ()
{
}

VectorXd
SplineStateConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());

  StateLinXd state_of_poly = active_poly_->GetPoint(t_local_);

  int row = 0; // constraint count
  for (auto dxdt :  derivatives_) {

    g.middleRows(row,n_dim_) = state_of_poly.GetByIndex(dxdt);
    row += n_dim_;
  }

  return g;
}

void
SplineStateConstraint::FillJacobianWithRespectTo (std::string var_set,
                                                  Jacobian& jac) const
{
  if (var_set == active_poly_->GetName()) {

    int row = 0;
    for (auto dxdt :  derivatives_) {

      Jacobian jac_deriv = active_poly_->GetJacobian(t_local_,dxdt);
      jac.middleRows(row,n_dim_) = jac_deriv;
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
                                    const VecTimes& poly_durations,
                                    const DerivativeVec& derivatives
                                    )
{
  SetName("SplineJunctionConstraint-" + spline_id);
  spline_ = Spline::BuildSpline(opt_vars, spline_id, poly_durations);
  derivatives_   = derivatives;
  n_dim_         = spline_->GetPoint(0.0).kNumDim;

  n_junctions_ = spline_->GetPolynomials().size()-1; // because one less junction than poly's.
  int n_constraints = derivatives_.size() * n_junctions_ * n_dim_;
  AddOptimizationVariables(opt_vars);
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

    auto p0 = spline_->GetPolynomial(id)->GetPoint(T);
    auto p1 = spline_->GetPolynomial(id+1)->GetPoint(0.0);

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
  int id=0;
  for (auto p : spline_->GetPolynomials()) {
    if (var_set == p->GetName()) {

      double T = spline_->GetDurationOfPoly(id);
      for (auto dxdt :  derivatives_) {

        auto jac_final = p->GetJacobian(T,dxdt);
        auto jac_start = p->GetJacobian(0.0,dxdt);

        if (id != 0) // start of first spline constrained elsewhere
          jac.middleRows(IndexRowStart(id,   Start, dxdt), n_dim_) = -jac_start;

        if (id != spline_->GetPolynomials().size()-1) // end of last spline constrained elsewhere
          jac.middleRows(IndexRowStart(id,   Final, dxdt), n_dim_) =  jac_final;

      }
    }

    id++;
  }
}

int
SplineJunctionConstraint::IndexRowStart (int spline_id, EvalTime which_end,
                                         MotionDerivative dxdt) const
{
  int constraints_per_junction = n_dim_*derivatives_.size();
  int junction = spline_id-which_end;
  return junction*constraints_per_junction + dxdt*n_dim_;
}

} /* namespace opt */
} /* namespace xpp */


