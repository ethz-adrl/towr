/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/dynamic_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

#include <xpp/endeffectors.h>
#include <xpp/opt/bound.h>
#include <xpp/opt/variables/variable_names.h>
#include <xpp/opt/variables/spline.h>
#include <xpp/state.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint (const OptVarsPtr& opt_vars,
                                      const DynamicModelPtr& m,
                                      const VecTimes& base_poly_durations,
                                      double T,
                                      double dt)
    :TimeDiscretizationConstraint(T, dt, opt_vars)
{
  model_ = m;

  SetName("DynamicConstraint");
  base_linear_  = Spline::BuildSpline(opt_vars, id::base_linear,  base_poly_durations);
  base_angular_ = Spline::BuildSpline(opt_vars, id::base_angular, base_poly_durations);

  for (auto ee : model_->GetEEIDs()) {
    ee_splines_.push_back(Spline::BuildSpline(opt_vars, id::GetEEId(ee), {}));
    ee_forces_.push_back(Spline::BuildSpline(opt_vars, id::GetEEForceId(ee), {}));
  }

  SetRows(GetNumberOfNodes()*kDim6d);

  converter_ = AngularStateConverter(base_angular_);
}

// zmp_ possibly implement as lookup-map, as in NodeValues?
int
DynamicConstraint::GetRow (int node, Coords6D dimension) const
{
  return kDim6d*node + dimension;
}

DynamicConstraint::~DynamicConstraint ()
{
}

void
DynamicConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{
  // acceleration the system should have given by physics
  UpdateModel(t);
  Vector6d acc_model = model_->GetBaseAcceleration();

  // acceleration base polynomial has with current values of optimization variables
  Vector6d acc_parametrization = Vector6d::Zero();
  acc_parametrization.middleRows(AX, kDim3d) = converter_.GetAngularAcceleration(t);
  acc_parametrization.middleRows(LX, kDim3d) = base_linear_->GetPoint(t).a_;

  for (auto dim : AllDim6D)
    g(GetRow(k,dim)) = acc_model(dim) - acc_parametrization(dim);
}

void
DynamicConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{
  for (auto dim : AllDim6D) {
    if (dim == LZ)
      bounds.at(GetRow(k,dim)) = Bound(kGravity, kGravity);
    else
      bounds.at(GetRow(k,dim)) = kEqualityBound_;
  }
}

void
DynamicConstraint::UpdateJacobianAtInstance(double t, int k, Jacobian& jac,
                                            std::string var_set) const
{
  UpdateModel(t);

  int n = jac.cols();
  Jacobian jac_model(kDim6d,n);
  Jacobian jac_parametrization(kDim6d,n);

  for (auto ee : model_->GetEEIDs()) {

    if (ee_forces_.at(ee)->DoVarAffectCurrentState(var_set,t)) {
      Jacobian jac_ee_force = ee_forces_.at(ee)->GetJacobian(t,kPos);
      jac_model = model_->GetJacobianofAccWrtForce(jac_ee_force, ee);
    }

    if (ee_splines_.at(ee)->DoVarAffectCurrentState(var_set,t)) {
      Jacobian jac_ee_pos = ee_splines_.at(ee)->GetJacobian(t,kPos);
      jac_model = model_->GetJacobianofAccWrtEEPos(jac_ee_pos, ee);
    }
  }

  if (base_linear_->DoVarAffectCurrentState(var_set,t)) {
    Jacobian jac_base_lin_pos = base_linear_->GetJacobian(t,kPos);
    jac_model = model_->GetJacobianOfAccWrtBaseLin(jac_base_lin_pos);
    jac_parametrization.middleRows(LX, kDim3d) = base_linear_->GetJacobian(t,kAcc);
  }

  if (base_angular_->DoVarAffectCurrentState(var_set,t)) {
    Jacobian jac_base_ang_pos = base_angular_->GetJacobian(t,kPos);
    jac_model = model_->GetJacobianOfAccWrtBaseAng(jac_base_ang_pos);
    jac_parametrization.middleRows(AX, kDim3d) = converter_.GetDerivOfAngAccWrtCoeff(t);
  }

  jac.middleRows(GetRow(k,AX), kDim6d) = jac_model - jac_parametrization;
}

void
DynamicConstraint::UpdateModel (double t) const
{
  auto com_pos = base_linear_->GetPoint(t).p_;

  int n_ee = model_->GetEEIDs().size();
  EndeffectorsPos ee_pos(n_ee);
  Endeffectors<Vector3d> ee_force(n_ee);
  for (auto ee :  ee_pos.GetEEsOrdered()) {
    ee_force.At(ee) = ee_forces_.at(ee)->GetPoint(t).p_;
    ee_pos.At(ee)   = ee_splines_.at(ee)->GetPoint(t).p_;
  }

  model_->SetCurrent(com_pos, ee_force, ee_pos);
}

} /* namespace opt */
} /* namespace xpp */

