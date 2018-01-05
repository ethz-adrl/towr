/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp_opt/motion_optimizer_facade.h>

#include <algorithm>
#include <cassert>
#include <deque>
#include <Eigen/Sparse>
#include <string>
#include <tuple>
#include <utility>

#include <xpp_opt/angular_state_converter.h>
#include <xpp_opt/cost_constraint_factory.h>
#include <xpp_opt/models/gait_generator.h>
#include <xpp_opt/models/kinematic_model.h>
#include <xpp_opt/polynomial.h>
#include <xpp_opt/variables/coeff_spline.h>
#include <xpp_opt/variables/contact_schedule.h>
#include <xpp_opt/variables/phase_nodes.h>
#include <xpp_opt/variables/variable_names.h>


namespace xpp {

using Composite = opt::Composite;

MotionOptimizerFacade::MotionOptimizerFacade ()
{
  params_ = std::make_shared<OptimizationParameters>();

//    model_.MakeMonopedModel();
//  model_.MakeBipedModel();
  model_.MakeAnymalModel();
//  model_.MakeHyqModel();

  model_.SetInitialState(inital_base_, initial_ee_W_);
//  RobotModel::SetAnymalInitialState(inital_base_, initial_ee_W_);
}

void
MotionOptimizerFacade::SetInitialState (const RobotStateCartesian& curr_state)
{
  inital_base_     = State3dEuler(); // first zero everything
  inital_base_.lin = curr_state.base_.lin;
//  motion_optimizer_.inital_base_.lin.v_.setZero();
//  motion_optimizer_.inital_base_.lin.a_.setZero();

  inital_base_.ang.p_ = GetUnique(GetEulerZYXAngles(curr_state.base_.ang.q));

//  kindr::RotationQuaternionD quat(curr_state.base_.ang.q);
//  kindr::EulerAnglesZyxD euler(quat);
//  euler.setUnique(); // to express euler angles close to 0,0,0, not 180,180,180 (although same orientation)
//  inital_base_.ang.p_ = euler.toImplementation().reverse();

  SetIntialFootholds(curr_state.ee_motion_.Get(kPos));
}

void
MotionOptimizerFacade::SetFinalState (const StateLin3d& lin,
                                      const StateLin3d& ang)
{
  final_base_.ang = ang;
  final_base_.lin = lin;

  // height depends on terrain
  double z_terrain = terrain_->GetHeight(lin.p_.x(), lin.p_.y());
  double z_nominal_B = model_.kinematic_model_->GetNominalStanceInBase().at(0).z();
  final_base_.lin.p_.z() = z_terrain - z_nominal_B;
}

opt::Problem
MotionOptimizerFacade::BuildNLP ()
{
  opt::Problem nlp;

  CostConstraintFactory factory;
  factory.Init(params_, terrain_, model_,
               initial_ee_W_, inital_base_, final_base_);

  for (auto c : factory.GetVariableSets())
    nlp.AddVariableSet(c);

  for (ConstraintName name : params_->GetUsedConstraints())
    for (auto c : factory.GetConstraint(name))
      nlp.AddConstraintSet(c);

  for (const auto& pair : params_->GetCostWeights())
    for (auto c : factory.GetCost(pair.first, pair.second))
      nlp.AddCostSet(c);

  return nlp;
}

std::vector<MotionOptimizerFacade::RobotStateVec>
MotionOptimizerFacade::GetIntermediateSolutions (opt::Problem& nlp, double dt) const
{
  std::vector<RobotStateVec> trajectories;

  for (int iter=0; iter<nlp.GetIterationCount(); ++iter) {
    auto opt_var = nlp.GetOptVariables(iter);
    trajectories.push_back(GetTrajectory(opt_var, dt));
  }

  return trajectories;
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (const VariablesCompPtr& vars,
                                      double dt) const
{
  RobotStateVec trajectory;
  double t=0.0;
  double T = params_->GetTotalTime();

  while (t<=T+1e-5) {

    RobotStateCartesian state(initial_ee_W_.GetEECount());

    state.base_.lin = vars->GetComponent<Spline>(id::base_linear)->GetPoint(t);
    state.base_.ang = AngularStateConverter::GetState(vars->GetComponent<Spline>(id::base_angular)->GetPoint(t));

    for (auto ee : state.ee_motion_.GetEEsOrdered()) {
//      state.ee_contact_.at(ee) = vars->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee))->IsInContact(t);
      auto ee_motioin = vars->GetComponent<PhaseNodes>(id::GetEEMotionId(ee));
      state.ee_contact_.at(ee) = ee_motioin->IsConstantPhase(t);
      state.ee_motion_.at(ee)  = ee_motioin->GetPoint(t);
      state.ee_forces_.at(ee)  = vars->GetComponent<Spline>(id::GetEEForceId(ee))->GetPoint(t).p_;
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += dt;
  }

  return trajectory;
}


/*! @brief Returns unique Euler angles in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
 *
 * Taken from https://github.com/ethz-asl/kindr
 *
 * Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
Vector3d
MotionOptimizerFacade::GetUnique (const Vector3d& zyx_non_unique) const
{
  Vector3d zyx = zyx_non_unique;
  const double tol = 1e-3;

  // wrap angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
  if(zyx.y() < -M_PI/2 - tol)
  {
    if(zyx.x() < 0) {
      zyx.x() = zyx.x() + M_PI;
    } else {
      zyx.x() = zyx.x() - M_PI;
    }

    zyx.y() = -(zyx.y() + M_PI);

    if(zyx.z() < 0) {
      zyx.z() = zyx.z() + M_PI;
    } else {
      zyx.z() = zyx.z() - M_PI;
    }
  }
  else if(-M_PI/2 - tol <= zyx.y() && zyx.y() <= -M_PI/2 + tol)
  {
    zyx.x() -= zyx.z();
    zyx.z() = 0;
  }
  else if(-M_PI/2 + tol < zyx.y() && zyx.y() < M_PI/2 - tol)
  {
    // ok
  }
  else if(M_PI/2 - tol <= zyx.y() && zyx.y() <= M_PI/2 + tol)
  {
    // todo: M_PI/2 should not be in range, other formula?
    zyx.x() += zyx.z();
    zyx.z() = 0;
  }
  else // M_PI/2 + tol < zyx.y()
  {
    if(zyx.x() < 0) {
      zyx.x() = zyx.x() + M_PI;
    } else {
      zyx.x() = zyx.x() - M_PI;
    }

    zyx.y() = -(zyx.y() - M_PI);

    if(zyx.z() < 0) {
      zyx.z() = zyx.z() + M_PI;
    } else {
      zyx.z() = zyx.z() - M_PI;
    }
  }

  return zyx;
}

} /* namespace xpp */
