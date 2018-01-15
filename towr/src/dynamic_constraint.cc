/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/constraints/dynamic_constraint.h>

#include <memory>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <towr/variables/variable_names.h>
#include <towr/variables/cartesian_dimensions.h>

namespace towr {


DynamicConstraint::DynamicConstraint (const DynamicModel::Ptr& m,
                                      const std::vector<double>& evaluation_times,
                                      const SplineHolder& spline_holder)
    :TimeDiscretizationConstraint(evaluation_times, "DynamicConstraint")
{
  model_ = m;
  gravity_ = m->GetGravityAcceleration();

  // link with up-to-date spline variables
  base_linear_  = spline_holder.GetBaseLinear();
  base_angular_ = spline_holder.GetBaseAngular();
  converter_    = AngularStateConverter(base_angular_);

  ee_forces_ = spline_holder.GetEEForce();
  ee_motion_ = spline_holder.GetEEMotion();

  n_ee_ = ee_motion_.size();

  SetRows(GetNumberOfNodes()*k6D);
}

int
DynamicConstraint::GetRow (int node, Dim6D dimension) const
{
  return k6D*node + dimension;
}

void
DynamicConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{
  // acceleration the system should have given by physics
  UpdateModel(t);
  Vector6d acc_model = model_->GetBaseAcceleration();

  // acceleration base polynomial has with current values of optimization variables
  Vector6d acc_parametrization = Vector6d::Zero();
  acc_parametrization.middleRows(AX, k3D) = converter_.GetAngularAcceleration(t);
  acc_parametrization.middleRows(LX, k3D) = base_linear_->GetPoint(t).a();

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
  Jacobian jac_model(k6D,n);
  Jacobian jac_parametrization(k6D,n);

  if (var_set == id::base_lin_nodes) {
    Jacobian jac_base_lin_pos = base_linear_->GetJacobianWrtNodes(t,kPos);
    jac_model = model_->GetJacobianOfAccWrtBaseLin(jac_base_lin_pos);
    jac_parametrization.middleRows(LX, k3D) = base_linear_->GetJacobianWrtNodes(t,kAcc);
  }

  if (var_set == id::base_ang_nodes) {
    Jacobian jac_ang_vel_wrt_coeff = converter_.GetDerivOfAngVelWrtEulerNodes(t);
//    Jacobian jac_base_ang_pos = base_angular_->GetJacobian(t,kPos);
    jac_model = model_->GetJacobianOfAccWrtBaseAng(jac_ang_vel_wrt_coeff);
    jac_parametrization.middleRows(AX, k3D) = converter_.GetDerivOfAngAccWrtEulerNodes(t);
  }


  for (int ee=0; ee<n_ee_; ++ee) {

    if (var_set == id::EEForceNodes(ee)) {
      Jacobian jac_ee_force = ee_forces_.at(ee)->GetJacobianWrtNodes(t,kPos);
      jac_model = model_->GetJacobianofAccWrtForce(jac_ee_force, ee);
    }

    if (var_set == id::EEMotionNodes(ee)) {
      Jacobian jac_ee_pos = ee_motion_.at(ee)->GetJacobianWrtNodes(t,kPos);
      jac_model = model_->GetJacobianofAccWrtEEPos(jac_ee_pos, ee);
    }

    // is only executed, if ee_timings part of optimization variables,
    // so otherwise the ee_timings_ pointer can actually be null.
    if (var_set == id::EESchedule(ee)) {

      Jacobian jac_f_dT = ee_forces_.at(ee)->GetJacobianOfPosWrtDurations(t);
      jac_model += model_->GetJacobianofAccWrtForce(jac_f_dT, ee);

      Jacobian jac_x_dT = ee_motion_.at(ee)->GetJacobianOfPosWrtDurations(t);
      jac_model +=  model_->GetJacobianofAccWrtEEPos(jac_x_dT, ee);
    }
  }


  jac.middleRows(GetRow(k,AX), k6D) = jac_model - jac_parametrization;
}

void
DynamicConstraint::UpdateModel (double t) const
{
  auto com_pos   = base_linear_->GetPoint(t).p();
  Vector3d omega = converter_.GetAngularVelocity(t);

//  int n_ee = model_->GetEEIDs().size();
  std::vector<Vector3d> ee_pos(n_ee_);
  std::vector<Vector3d> ee_force(n_ee_);
  for (int ee=0; ee<n_ee_; ++ee) {
    ee_force.at(ee) = ee_forces_.at(ee)->GetPoint(t).p();
    ee_pos.at(ee)   = ee_motion_.at(ee)->GetPoint(t).p();
  }

  model_->SetCurrent(com_pos, omega, ee_force, ee_pos);
}

} /* namespace towr */
