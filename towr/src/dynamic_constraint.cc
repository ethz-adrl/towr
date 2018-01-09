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


DynamicConstraint::DynamicConstraint (const RobotModel& m,
                                      const std::vector<double>& evaluation_times,
                                      const OptimizationParameters& params)
    :TimeDiscretizationConstraint(evaluation_times, "DynamicConstraint")
{
  model_ = m.dynamic_model_;
  gravity_ = m.dynamic_model_->GetGravityAcceleration();
  optimize_timings_ = params.OptimizeTimings();
  base_poly_durations_ = params.GetBasePolyDurations();

  for (auto ee : model_->GetEEIDs()) {
    ee_phase_durations_.push_back(m.gait_generator_->GetContactSchedule(params.GetTotalTime(), ee));
  }

  SetRows(GetNumberOfNodes()*kDim6d);
}

// smell make a common function, maybe in derived class where this is
// done for all constraint classes, b/c often repeats itself
void
DynamicConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
//  base_linear_  = x->GetComponent<Spline>(id::base_linear);
//  base_angular_ = x->GetComponent<Spline>(id::base_angular);

  auto base_linear_nodes = x->GetComponent<NodeValues>(id::base_linear);
  base_linear_ = std::make_shared<Spline>(base_linear_nodes, base_poly_durations_);
  base_linear_nodes->AddObserver(base_linear_);

  auto base_angular_nodes = x->GetComponent<NodeValues>(id::base_angular);
  base_angular_ = std::make_shared<Spline>(base_angular_nodes, base_poly_durations_);
  base_angular_nodes->AddObserver(base_angular_);



  for (auto ee : model_->GetEEIDs()) {


    auto ee_motion_nodes = x->GetComponent<NodeValues>(id::GetEEMotionId(ee));
    auto ee_spline = std::make_shared<Spline>(ee_motion_nodes, ee_phase_durations_.at(ee));
    ee_motion_.push_back(ee_spline);
    ee_motion_nodes->AddObserver(ee_spline);
//    ee_motion_.push_back(x->GetComponent<Spline>(id::GetEEMotionId(ee)));

    auto ee_forces_nodes = x->GetComponent<NodeValues>(id::GetEEForceId(ee));
    auto ee_force_spline = std::make_shared<Spline>(ee_forces_nodes, ee_phase_durations_.at(ee));
    ee_forces_.push_back(ee_force_spline);
    ee_forces_nodes->AddObserver(ee_force_spline);
//    ee_forces_.push_back(x->GetComponent<Spline>(id::GetEEForceId(ee)));

    if (optimize_timings_) {
      auto contact_schedule = x->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee));
      ee_timings_.push_back(contact_schedule); // only need this for Jacobian, but not really

      // smell dependes on order (must be this way)
      ee_motion_.at(ee)->SetContactSchedule(contact_schedule);
      contact_schedule->AddObserver(ee_motion_.at(ee));

      ee_forces_.at(ee)->SetContactSchedule(contact_schedule);
      contact_schedule->AddObserver(ee_forces_.at(ee));
    }

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
  using namespace ifopt;

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
    // so otherwise the ee_timings_ pointer can actually be null.
    if (var_set == id::GetEEScheduleId(ee)) {

      Jacobian jac_f_dT = ee_forces_.at(ee)->GetJacobianOfPosWrtDurations(t);
      jac_model += model_->GetJacobianofAccWrtForce(jac_f_dT, ee);

      Jacobian jac_x_dT = ee_motion_.at(ee)->GetJacobianOfPosWrtDurations(t);
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
