/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <towr/towr.h>

#include <ifopt/solvers/ipopt_adapter.h>
#include <ifopt/solvers/snopt_adapter.h>

#include <towr/nlp_factory.h>
#include <towr/variables/variable_names.h>
#include <towr/variables/angular_state_converter.h>
#include <towr/variables/spline.h>
#include <towr/variables/phase_nodes.h>
#include <towr/variables/contact_schedule.h>


namespace towr {

using namespace xpp;

void
TOWR::SetInitialState (const RobotStateCartesian& curr_state)
{
  inital_base_     = State3dEuler(); // first zero everything
  inital_base_.lin = curr_state.base_.lin;
  inital_base_.ang.p_ = GetUnique(GetEulerZYXAngles(curr_state.base_.ang.q));
  initial_ee_W_ = curr_state.ee_motion_.Get(kPos);
}

void TOWR::SetParameters(const State3dEuler& final_base,
                                          double total_time,
                                          const RobotModel& model,
                                          HeightMap::Ptr terrain)
{
  final_base_ = final_base;
//  // height depends on terrain
//  double z_terrain = terrain_->GetHeight(lin.p_.x(), lin.p_.y());
//  double z_nominal_B = model_.kinematic_model_->GetNominalStanceInBase().at(0).z();
//  final_base_.lin.p_.z() = z_terrain - z_nominal_B;

  params_.SetTotalDuration(total_time);

  model_ = model;

  terrain_ = terrain;
  SetTerrainHeightFromAvgFootholdHeight(terrain_); // make sure initial footholds are set
}

ifopt::Problem
TOWR::BuildNLP () const
{
  ifopt::Problem nlp;

  NlpFactory factory;
  factory.Init(params_, terrain_, model_, initial_ee_W_, inital_base_, final_base_);

  for (auto c : factory.GetVariableSets())
    nlp.AddVariableSet(c);

  for (ConstraintName name : params_.GetUsedConstraints())
    for (auto c : factory.GetConstraint(name))
      nlp.AddConstraintSet(c);

  for (const auto& pair : params_.GetCostWeights())
    for (auto c : factory.GetCost(pair.first, pair.second))
      nlp.AddCostSet(c);

  return nlp;
}

void TOWR::SolveNLP()
{
  nlp_ = BuildNLP();

  ifopt::IpoptAdapter::Solve(nlp_);
  // opt::SnoptAdapter::Solve(nlp_);

  nlp_.PrintCurrent();
}

std::vector<TOWR::RobotStateVec>
TOWR::GetIntermediateSolutions (double dt) const
{
  std::vector<RobotStateVec> trajectories;

  for (int iter=0; iter<nlp_.GetIterationCount(); ++iter) {
    auto opt_var = nlp_.GetOptVariables(iter);
    trajectories.push_back(GetTrajectory(opt_var, dt));
  }

  return trajectories;
}

TOWR::RobotStateVec
TOWR::GetTrajectory (double dt) const
{
  return GetTrajectory(nlp_.GetOptVariables(), dt);
}

TOWR::RobotStateVec
TOWR::GetTrajectory (const VariablesCompPtr& x, double dt) const
{
  RobotStateVec trajectory;
  double t=0.0;
  double T = params_.GetTotalTime();


  auto base_linear_nodes_ = x->GetComponent<NodeValues>(id::base_linear);
  auto base_linear_ = std::make_shared<Spline>(base_linear_nodes_, params_.GetBasePolyDurations());

  auto base_angular_nodes_ = x->GetComponent<NodeValues>(id::base_angular);
  auto base_angular_ = std::make_shared<Spline>(base_angular_nodes_, params_.GetBasePolyDurations());


  std::vector<Spline::Ptr> ee_motions;
  std::vector<Spline::Ptr> ee_forces;


  // reconstruct the spline from the optimization variables
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {

    std::vector<double> ee_phase_durations = model_.gait_generator_->GetContactSchedule(params_.GetTotalTime(), ee);

    auto ee_motion_nodes = x->GetComponent<NodeValues>(id::GetEEMotionId(ee));
    auto ee_spline = std::make_shared<Spline>(ee_motion_nodes, ee_phase_durations);
    ee_motions.push_back(ee_spline);
    ee_motion_nodes->AddObserver(ee_spline);

    auto ee_forces_nodes = x->GetComponent<NodeValues>(id::GetEEForceId(ee));
    auto ee_force_spline = std::make_shared<Spline>(ee_forces_nodes, ee_phase_durations);
    ee_forces.push_back(ee_force_spline);
    ee_forces_nodes->AddObserver(ee_force_spline);

    if (params_.OptimizeTimings()) {


      auto contact_schedule = x->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee));

      ee_motions.at(ee)->SetContactSchedule(contact_schedule);
      contact_schedule->AddObserver(ee_motions.at(ee));

      ee_forces.at(ee)->SetContactSchedule(contact_schedule);
      contact_schedule->AddObserver(ee_forces.at(ee));
    }
  }



  // get the values
  while (t<=T+1e-5) {

    RobotStateCartesian state(initial_ee_W_.GetEECount());


//    state.base_.lin = vars->GetComponent<Spline>(id::base_linear)->GetPoint(t);
//    state.base_.ang = AngularStateConverter::GetState(vars->GetComponent<Spline>(id::base_angular)->GetPoint(t));


    state.base_.lin = base_linear_->GetPoint(t);
    state.base_.ang = AngularStateConverter::GetState(base_angular_->GetPoint(t));



    for (auto ee : state.ee_motion_.GetEEsOrdered()) {
//      state.ee_contact_.at(ee) = vars->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee))->IsInContact(t);
      // smell find way to get back contact state, possibly without needing ContactSchedule
//      state.ee_contact_.at(ee) = ee_motion->IsConstantPhase(t);
//      auto ee_motion_nodes = x->GetComponent<NodeValues>(id::GetEEMotionId(ee));
//      auto ee_motion = std::make_shared<Spline>(ee_motion_nodes, model_.gait_generator_->GetContactSchedule(params_.GetTotalTime(), ee));
//
//      if (params_.OptimizeTimings())
//        x->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee))->AddObserver(ee_motion);

//      auto ee_motion = vars->GetComponent<Spline>(id::GetEEMotionId(ee));
      state.ee_motion_.at(ee)  = ee_motions.at(ee)->GetPoint(t);
      state.ee_forces_.at(ee)  = ee_forces.at(ee)->GetPoint(t).p_;
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += dt;
  }

  return trajectory;
}

void TOWR::SetTerrainHeightFromAvgFootholdHeight(
    HeightMap::Ptr& terrain) const
{
  double avg_height=0.0;
  for ( auto pos : initial_ee_W_.ToImpl())
    avg_height += pos.z()/initial_ee_W_.GetEECount();
  terrain->SetGroundHeight(avg_height);
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
TOWR::GetUnique (const Vector3d& zyx_non_unique) const
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

} /* namespace towr */
