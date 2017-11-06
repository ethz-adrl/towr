/**
 @file    spline_junction_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 5, 2017
 @brief   Brief description
 */

#include <xpp_opt/constraints/spline_constraint.h>

#include <algorithm>
#include <Eigen/Dense>

#include <xpp_opt/variables/spline.h>

namespace xpp {

SplineStateConstraint::SplineStateConstraint (const OptVarsPtr& opt_vars,
                                              const std::string& id,
                                              double t_global,
                                              const StateLinXd& state,
                                              const DerivativeVec& derivatives,
                                              const Dimensions& dimensions)
    :Constraint(opt_vars)
{

  // print out names correctly
  std::string deriv_string;
  for (auto d : derivatives) {
    if (d==kPos)
      deriv_string += "pos,";
    if (d==kVel)
      deriv_string += "vel,";
    if (d==kAcc)
      deriv_string += "acc,";
  }

  std::string dim_string;
  for (auto d : dimensions) {
    if (d==X)
      deriv_string += "X,";
    if (d==Y)
      deriv_string += "Y,";
    if (d==Z)
      deriv_string += "Z,";
  }


  SetName(id + ",t=" + std::to_string(t_global) + ", " + deriv_string + dim_string);

  spline_      = opt_vars->GetComponent<Spline>(id);
  t_global_    = t_global;

  state_desired_ = state;
  derivatives_   = derivatives;
  dims_          = dimensions;

  int n_constraints = derivatives.size()*dims_.size();
//  AddOptimizationVariables(opt_vars);
  SetRows(n_constraints);
}

SplineStateConstraint::~SplineStateConstraint ()
{
}

VectorXd
SplineStateConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());

  StateLinXd state_of_poly = spline_->GetPoint(t_global_);

  int row = 0; // constraint count
  for (auto dxdt :  derivatives_)
    for (auto dim :  dims_)
      g(row++) = state_of_poly.GetByIndex(dxdt)(dim);

  return g;
}

void
SplineStateConstraint::FillJacobianWithRespectTo (std::string var_set,
                                                  Jacobian& jac) const
{
  if (spline_->HoldsVarsetThatIsActiveNow(var_set, t_global_)) {
    int row = 0;
    for (auto dxdt :  derivatives_) {

      Jacobian jac_deriv = spline_->GetJacobian(t_global_, dxdt);
      for (auto dim :  dims_) {
        jac.row(row++) = jac_deriv.row(dim);
      }
    }
  }
}

VecBound
SplineStateConstraint::GetBounds () const
{
  VecBound bounds;

  for (auto dxdt :  derivatives_) {
    VectorXd state = state_desired_.GetByIndex(dxdt);
    for (auto dim :  dims_)
      bounds.push_back(NLPBound(state(dim), state(dim)));
  }

  return bounds;
}






SplineJunctionConstraint::SplineJunctionConstraint (const OptVarsPtr& opt_vars,
                                                    const std::string& spline_id,
                                                    const DerivativeVec& derivatives
                                                    )
    : Constraint(opt_vars)
{
  SetName("SplineJunctionConstraint-" + spline_id);

  // need specific functions from coefficient spline
  spline_        = opt_vars->GetComponent<CoeffSpline>(spline_id);
  derivatives_   = derivatives;
  n_dim_         = spline_->GetPoint(0.0).kNumDim;

  n_junctions_ = spline_->GetPolyCount()-1; // because one less junction than poly's.
  SetRows(derivatives_.size() * n_junctions_ * n_dim_);
//  AddOptimizationVariables(opt_vars);
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

void
SplineJunctionConstraint::FillJacobianWithRespectTo (std::string var_set,
                                                     Jacobian& jac) const
{
  int row = 0;
  for (int id=0; id<n_junctions_; ++id) {

    auto vars_before = spline_->GetVarSet(id);
    double T_before  = spline_->GetDurationOfPoly(id);

    auto vars_after  = spline_->GetVarSet(id+1);

    for (auto dxdt :  derivatives_) {

      if (var_set == vars_before->GetName())
        jac.middleRows(row, n_dim_) =  vars_before->GetJacobian(T_before,dxdt);

      if (var_set == vars_after->GetName())
        jac.middleRows(row, n_dim_) = -vars_after->GetJacobian(0.0,dxdt);

      row += n_dim_;

    }
  }
}

VecBound
SplineJunctionConstraint::GetBounds () const
{
  VecBound bounds(GetRows());
  std::fill(bounds.begin(), bounds.end(), BoundZero);

  return bounds;
}

} /* namespace xpp */


