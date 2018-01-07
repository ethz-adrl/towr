/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <towr/constraints/dynamic_constraint.h>

#include <memory>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp_states/endeffectors.h>
#include <xpp_states/state.h>

#include <towr/variables/variable_names.h>

namespace towr {

using namespace xpp;


DynamicConstraint::DynamicConstraint (const DynamicModel::Ptr& m,
                                      const std::vector<double>& evaluation_times,
                                      bool optimize_timings)
    :TimeDiscretizationConstraint(evaluation_times, "DynamicConstraint")
{
  model_ = m;
  gravity_ = m->GetGravityAcceleration();
  optimize_timings_ = optimize_timings;

  SetRows(GetNumberOfNodes()*kDim6d);
}

void
towr::DynamicConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  base_linear_  = x->GetComponent<Spline>(id::base_linear);
  base_angular_ = x->GetComponent<Spline>(id::base_angular);

  for (auto ee : model_->GetEEIDs()) {
    ee_motion_.push_back(x->GetComponent<NodeValues>(id::GetEEMotionId(ee)));
    ee_forces_.push_back(x->GetComponent<NodeValues>(id::GetEEForceId(ee)));
    if (optimize_timings_)
      ee_timings_.push_back(x->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee)));
  }

  converter_ = AngularStateConverter(base_angular_);
}

int
DynamicConstraint::GetRow (int node, Coords6D dimension) const
{
  return kDim6d*node + dimension;
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
  using namespace opt;

  for (auto dim : AllDim6D) {
    if (dim == LZ)
      bounds.at(GetRow(k,dim)) = Bounds(gravity_, gravity_);
    else
      bounds.at(GetRow(k,dim)) = BoundZero;
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

  if (var_set == id::base_linear) {
    Jacobian jac_base_lin_pos = base_linear_->GetJacobian(t,kPos);
    jac_model = model_->GetJacobianOfAccWrtBaseLin(jac_base_lin_pos);
    jac_parametrization.middleRows(LX, kDim3d) = base_linear_->GetJacobian(t,kAcc);
  }

  if (var_set == id::base_angular) {
    Jacobian jac_ang_vel_wrt_coeff = converter_.GetDerivOfAngVelWrtCoeff(t);
//    Jacobian jac_base_ang_pos = base_angular_->GetJacobian(t,kPos);
    jac_model = model_->GetJacobianOfAccWrtBaseAng(jac_ang_vel_wrt_coeff);
    jac_parametrization.middleRows(AX, kDim3d) = converter_.GetDerivOfAngAccWrtCoeff(t);
  }


  for (auto ee : model_->GetEEIDs()) {

    if (var_set == ee_forces_.at(ee)->GetName()) {
      Jacobian jac_ee_force = ee_forces_.at(ee)->GetJacobian(t,kPos);
      jac_model = model_->GetJacobianofAccWrtForce(jac_ee_force, ee);
    }

    if (var_set == ee_motion_.at(ee)->GetName()) {
      Jacobian jac_ee_pos = ee_motion_.at(ee)->GetJacobian(t,kPos);
      jac_model = model_->GetJacobianofAccWrtEEPos(jac_ee_pos, ee);
    }

    // is only executed, if ee_timings part of optimization variables,
    // so otherwise the pointer can actually be null.
    if (var_set == id::GetEEScheduleId(ee)) {

      Jacobian jac_f_dT = ee_timings_.at(ee)->GetJacobianOfPos(t, id::GetEEForceId(ee));
      jac_model += model_->GetJacobianofAccWrtForce(jac_f_dT, ee);

      Jacobian jac_x_dT = ee_timings_.at(ee)->GetJacobianOfPos(t, id::GetEEMotionId(ee));
      jac_model +=  model_->GetJacobianofAccWrtEEPos(jac_x_dT, ee);
    }
  }


  jac.middleRows(GetRow(k,AX), kDim6d) = jac_model - jac_parametrization;
}

void
DynamicConstraint::UpdateModel (double t) const
{
  auto com_pos   = base_linear_->GetPoint(t).p_;
  Vector3d omega = converter_.GetAngularVelocity(t);

  int n_ee = model_->GetEEIDs().size();
  EndeffectorsPos ee_pos(n_ee);
  Endeffectors<Vector3d> ee_force(n_ee);
  for (auto ee :  ee_pos.GetEEsOrdered()) {
    ee_force.at(ee) = ee_forces_.at(ee)->GetPoint(t).p_;
    ee_pos.at(ee)   = ee_motion_.at(ee)->GetPoint(t).p_;
  }

  model_->SetCurrent(com_pos, omega, ee_force, ee_pos);
}

} /* namespace towr */
