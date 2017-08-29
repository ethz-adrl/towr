/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/constraints/dynamic_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/endeffectors.h>
#include <xpp/state.h>
#include <xpp/variables/variable_names.h>

namespace xpp {
namespace opt {


DynamicConstraint::DynamicConstraint (const OptVarsPtr& opt_vars,
                                      const DynamicModel::Ptr& m,
                                      const std::vector<double>& evaluation_times)
    :TimeDiscretizationConstraint(opt_vars)
{
  dts_ = evaluation_times;
  model_ = m;
  gravity_ = m->GetGravityAcceleration();


  SetName("DynamicConstraint");
  base_linear_  = opt_vars->GetComponent<Spline>(id::base_linear);
  base_angular_ = opt_vars->GetComponent<Spline>(id::base_angular);

  for (auto ee : model_->GetEEIDs()) {
    ee_motion_.push_back(opt_vars->GetComponent<NodeValues>(id::GetEEMotionId(ee)));
    ee_forces_.push_back(opt_vars->GetComponent<NodeValues>(id::GetEEForceId(ee)));
    ee_timings_.push_back(opt_vars->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee)));
  }

  SetRows(GetNumberOfNodes()*kDim6d);
  converter_ = AngularStateConverter(base_angular_);
}

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
      bounds.at(GetRow(k,dim)) = Bound(gravity_, gravity_);
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

    if (var_set == ee_forces_.at(ee)->GetName()) {
      Jacobian jac_ee_force = ee_forces_.at(ee)->GetJacobian(t,kPos);
      jac_model = model_->GetJacobianofAccWrtForce(jac_ee_force, ee);
    }

    if (var_set == ee_motion_.at(ee)->GetName()) {
      Jacobian jac_ee_pos = ee_motion_.at(ee)->GetJacobian(t,kPos);
      jac_model = model_->GetJacobianofAccWrtEEPos(jac_ee_pos, ee);
    }

    if (var_set == ee_timings_.at(ee)->GetName()) {
      Jacobian jac_f_dT = ee_timings_.at(ee)->GetJacobianOfPos(t, id::GetEEForceId(ee));
      jac_model += model_->GetJacobianofAccWrtForce(jac_f_dT, ee);

      Jacobian jac_x_dT = ee_timings_.at(ee)->GetJacobianOfPos(t, id::GetEEMotionId(ee));
      jac_model +=  model_->GetJacobianofAccWrtEEPos(jac_x_dT, ee);
    }
  }

  if (base_linear_->HoldsVarsetThatIsActiveNow(var_set,t)) {
    Jacobian jac_base_lin_pos = base_linear_->GetJacobian(t,kPos);
    jac_model = model_->GetJacobianOfAccWrtBaseLin(jac_base_lin_pos);
    jac_parametrization.middleRows(LX, kDim3d) = base_linear_->GetJacobian(t,kAcc);
  }

  if (base_angular_->HoldsVarsetThatIsActiveNow(var_set,t)) {
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
    ee_pos.At(ee)   = ee_motion_.at(ee)->GetPoint(t).p_;
  }

  model_->SetCurrent(com_pos, ee_force, ee_pos);
}

} /* namespace opt */
} /* namespace xpp */

