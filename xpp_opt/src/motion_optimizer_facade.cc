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

#include <opt_solve/solvers/ipopt_adapter.h>
#include <opt_solve/solvers/snopt_adapter.h>

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

using namespace opt;

MotionOptimizerFacade::MotionOptimizerFacade ()
{
  params_ = std::make_shared<OptimizationParameters>();

  RobotModel model;
  model.MakeAnymalModel();
  RobotModel::SetAnymalInitialState(inital_base_, initial_ee_W_);
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

MotionOptimizerFacade::VariablesCompPtr
MotionOptimizerFacade::BuildVariables () const
{
  auto opt_variables = std::make_shared<Composite>("nlp_variables", false);
  opt_variables->ClearComponents();


  switch (params_->GetBaseRepresentation()) {
    case OptimizationParameters::CubicHermite:
      SetBaseRepresentationHermite(opt_variables);
      break;
    case OptimizationParameters::PolyCoeff:
      SetBaseRepresentationCoeff(opt_variables);
      break;
    default:
      assert(false); // representation not defined
      break;
  }


  std::vector<std::shared_ptr<ContactSchedule>> contact_schedule;
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {
    contact_schedule.push_back(std::make_shared<ContactSchedule>(ee,
                                                                 params_->GetTotalTime(),
                                                                 model_.gait_generator_->GetNormalizedContactSchedule(ee),
                                                                 model_.gait_generator_->IsInContactAtStart(ee),
                                                                 params_->min_phase_duration_,
                                                                 params_->max_phase_duration_));
  }


  // Endeffector Motions
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {
    auto ee_motion = std::make_shared<EEMotionNodes>(contact_schedule.at(ee)->GetContactSequence(),
                                                     id::GetEEMotionId(ee),
                                                     params_->ee_splines_per_swing_phase_);

    double yaw = final_base_.ang.p_.z();
    Eigen::Matrix3d w_R_b = GetQuaternionFromEulerZYX(yaw, 0.0, 0.0).toRotationMatrix();
    Vector3d final_ee_pos_W = final_base_.lin.p_ + w_R_b*model_.kinematic_model_->GetNominalStanceInBase().at(ee);



    ee_motion->InitializeVariables(initial_ee_W_.at(ee), final_ee_pos_W, contact_schedule.at(ee)->GetTimePerPhase());

    // actually initial Z position should be constrained as well...-.-
    ee_motion->AddStartBound(kPos, {X,Y}, initial_ee_W_.at(ee));

    bool step_taken = ee_motion->GetNodes().size() > 2;
    if (step_taken) // otherwise overwrites start bound
      ee_motion->AddFinalBound(kPos, {X,Y}, final_ee_pos_W);

    opt_variables->AddComponent(ee_motion);
  }

  // Endeffector Forces
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {
    auto nodes_forces = std::make_shared<EEForceNodes>(contact_schedule.at(ee)->GetContactSequence(),
                                                     id::GetEEForceId(ee),
                                                     params_->force_splines_per_stance_phase_);

    Vector3d f_stance(0.0, 0.0, model_.dynamic_model_->GetStandingZForce());
    nodes_forces->InitializeVariables(f_stance, f_stance, contact_schedule.at(ee)->GetTimePerPhase());
    opt_variables->AddComponent(nodes_forces);
  }


  // make endeffector motion and forces dependent on durations
  bool optimize_timings = params_->ConstraintExists(TotalTime);
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {

    if (optimize_timings) {
      contact_schedule.at(ee)->AddObserver(opt_variables->GetComponent<PhaseNodes>(id::GetEEMotionId(ee)));
      contact_schedule.at(ee)->AddObserver(opt_variables->GetComponent<PhaseNodes>(id::GetEEForceId(ee)));
    } else {
      contact_schedule.at(ee)->SetRows(0); // zero rows means no variables to optimize
    }

    opt_variables->AddComponent(contact_schedule.at(ee));
  }


//  opt_variables->Print();
  return opt_variables;
}

void
MotionOptimizerFacade::SetBaseRepresentationCoeff (VariablesCompPtr& opt_variables) const
{
  int n_dim = inital_base_.lin.kNumDim;
  int order = params_->order_coeff_polys_;

  std::vector<double> base_spline_timings_ = params_->GetBasePolyDurations();
  auto coeff_spline_ang = std::make_shared<CoeffSpline>(id::base_angular, base_spline_timings_);
  for (int i=0; i<base_spline_timings_.size(); ++i) {
    auto p_ang = std::make_shared<Polynomial>(order, n_dim);
    auto var = std::make_shared<PolynomialVars>(id::base_angular+std::to_string(i), p_ang);
    opt_variables->AddComponent(var);

    coeff_spline_ang->poly_vars_.push_back(var);
  }
  coeff_spline_ang->InitializeVariables(inital_base_.ang.p_, final_base_.ang.p_);
  opt_variables->AddComponent(coeff_spline_ang); // add just for easy access later


  auto coeff_spline_lin = std::make_shared<CoeffSpline>(id::base_linear, base_spline_timings_);
  for (int i=0; i<base_spline_timings_.size(); ++i) {
    auto p_lin = std::make_shared<Polynomial>(order, n_dim);
    auto var = std::make_shared<PolynomialVars>(id::base_linear+std::to_string(i), p_lin);
    opt_variables->AddComponent(var);
    coeff_spline_lin->poly_vars_.push_back(var);
  }
  coeff_spline_lin->InitializeVariables(inital_base_.lin.p_, final_base_.lin.p_);
  opt_variables->AddComponent(coeff_spline_lin); // add just for easy access later
}

void
MotionOptimizerFacade::SetBaseRepresentationHermite (VariablesCompPtr& opt_variables_) const
{
  int n_dim = inital_base_.lin.kNumDim;
  std::vector<double> base_spline_timings_ = params_->GetBasePolyDurations();

  auto linear  = std::make_tuple(id::base_linear,  inital_base_.lin, final_base_.lin);
  auto angular = std::make_tuple(id::base_angular, inital_base_.ang, final_base_.ang);

  for (auto tuple : {linear, angular}) {
    std::string id   = std::get<0>(tuple);
    StateLin3d init  = std::get<1>(tuple);
    StateLin3d final = std::get<2>(tuple);

    auto spline = std::make_shared<NodeValues>(init.kNumDim,  base_spline_timings_.size(), id);
    spline->InitializeVariables(init.p_, final.p_, base_spline_timings_);

    std::vector<int> dimensions = {X,Y,Z};
    spline->AddStartBound(kPos, dimensions, init.p_);
    spline->AddStartBound(kVel, dimensions, init.v_);

    spline->AddFinalBound(kVel, dimensions, final.v_);

    if (id == id::base_linear) {
      spline->AddFinalBound(kPos, {X,Y}, final.p_); // only xy, z given by terrain
      //      spline->SetBoundsAboveGround();
    }
    if (id == id::base_angular)
      spline->AddFinalBound(kPos, {Z}, final.p_); // roll, pitch, yaw bound



    //    // force intermediate jump
    //    if (id == id::base_linear) {
    //      Vector3d inter = (init.p_ + final.p_)/2.;
    //      inter.z() = 0.8;
    //      spline->AddIntermediateBound(kPos, inter);
    //    }
    //
    //    if (id == id::base_angular) {
    //      spline->AddIntermediateBound(kPos, Vector3d::Zero());
    //    }


    opt_variables_->AddComponent(spline);
  }


//  auto spline_lin = std::make_shared<NodeValues>(n_dim,  base_spline_timings_.size(), id::base_linear);
//  spline_lin->InitializeVariables(inital_base_.lin.p_, final_base_.lin.p_, base_spline_timings_);
//  spline_lin->AddBound(0,   kPos, inital_base_.lin.p_);
//  spline_lin->AddBound(0,   kVel, inital_base_.lin.v_);
//  spline_lin->AddFinalBound(kPos,  final_base_.lin.p_);
//  spline_lin->AddFinalBound(kVel,  final_base_.lin.v_);
//  opt_variables_->AddComponent(spline_lin);
//
//
//  auto spline_ang = std::make_shared<NodeValues>(n_dim,  base_spline_timings_.size(), id::base_angular);
//  spline_ang->InitializeVariables(inital_base_.ang.p_, final_base_.ang.p_, base_spline_timings_);
//  spline_ang->AddBound(0,   kPos, inital_base_.ang.p_);
//  spline_ang->AddBound(0,   kVel, inital_base_.ang.v_);
//  spline_ang->AddFinalBound(kPos,  final_base_.ang.p_);
//  spline_ang->AddFinalBound(kVel,  final_base_.ang.v_);
//  opt_variables_->AddComponent(spline_ang);
}

void
MotionOptimizerFacade::BuildCostConstraints(const VariablesCompPtr& opt_variables)
{
  CostConstraintFactory factory;
  factory.Init(opt_variables, params_, terrain_, model_,
               initial_ee_W_, inital_base_, final_base_);

  auto constraints = std::make_unique<Composite>("constraints", false);
  for (ConstraintName name : params_->GetUsedConstraints())
    constraints->AddComponent(factory.GetConstraint(name));

//  constraints->Print();
  nlp.SetConstraints(std::move(constraints));


  auto costs = std::make_unique<Composite>("costs", true);
  for (const auto& pair : params_->GetCostWeights())
    costs->AddComponent(factory.GetCost(pair.first, pair.second));

//  costs->Print();
  nlp.SetCosts(std::move(costs));
}

void
MotionOptimizerFacade::SolveProblem ()
{
  auto variables = BuildVariables();
  nlp.SetVariables(variables);

  BuildCostConstraints(variables);

  switch (nlp_solver_) {
    case Ipopt: IpoptAdapter::Solve(nlp); break;
    case Snopt: SnoptAdapter::Solve(nlp); break;
    default: assert(false); // solver not implemented
  }

  nlp.PrintCurrent();
}

std::vector<MotionOptimizerFacade::RobotStateVec>
MotionOptimizerFacade::GetIntermediateSolutions (double dt) const
{
  std::vector<RobotStateVec> trajectories;

  for (int iter=0; iter<nlp.GetIterationCount(); ++iter) {
    auto opt_var = nlp.GetOptVariables(iter);
    auto vars_composite = std::dynamic_pointer_cast<Composite>(opt_var);
    trajectories.push_back(GetTrajectory(vars_composite, dt));
  }

  return trajectories;
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt) const
{
  auto opt_var = nlp.GetOptVariables();
  auto vars_composite = std::dynamic_pointer_cast<Composite>(opt_var);
  return GetTrajectory(vars_composite, dt);
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
      state.ee_contact_.at(ee) = vars->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee))->IsInContact(t);
      state.ee_motion_.at(ee)  = vars->GetComponent<Spline>(id::GetEEMotionId(ee))->GetPoint(t);
      state.ee_forces_.at(ee)  = vars->GetComponent<Spline>(id::GetEEForceId(ee))->GetPoint(t).p_;
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += dt;
  }

  return trajectory;
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
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
