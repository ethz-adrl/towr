/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/opt/motion_optimizer_facade.h>

#include <algorithm>
#include <cassert>
#include <deque>
#include <Eigen/Dense>
#include <string>
#include <utility>

#include <xpp/endeffectors.h>

#include <xpp/opt/angular_state_converter.h>
#include <xpp/opt/cost_constraint_factory.h>
#include <xpp/opt/variables/contact_schedule.h>
#include <xpp/opt/variables/spline.h>
#include <xpp/opt/variables/node_values.h>
#include <xpp/opt/variables/variable_names.h>
#include <xpp/opt/quadruped_motion_parameters.h>

#include <xpp/opt/ipopt_adapter.h>
#include <xpp/opt/snopt_adapter.h>

namespace xpp {
namespace opt {

MotionOptimizerFacade::MotionOptimizerFacade ()
{
  motion_parameters_ =  std::make_shared<opt::quad::SingleMotionParameters>();
  BuildDefaultStartStance();
  opt_variables_ = std::make_shared<Composite>("nlp_variables", true);
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
}

void
MotionOptimizerFacade::BuildDefaultStartStance ()
{
  double offset_x = 0.0;
  inital_base_.lin.p_ << offset_x+0.000350114, -1.44379e-7, 0.573311;
  inital_base_.lin.v_ << 0.000137518, -4.14828e-07,  0.000554118;
  inital_base_.lin.a_ << 0.000197966, -5.72241e-07, -5.13328e-06;

  inital_base_.ang.p_ << 0.0, 0.0, 0.0; // euler (roll, pitch, yaw)

  initial_ee_W_ = motion_parameters_->GetNominalStanceInBase();
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {
    initial_ee_W_.At(ee) += inital_base_.lin.p_;
    initial_ee_W_.At(ee).z() = 0.0;
  }
}

void
MotionOptimizerFacade::BuildVariables ()
{
  opt_variables_->ClearComponents();


  std::vector<std::shared_ptr<ContactSchedule>> contact_schedule;
  for (auto ee : motion_parameters_->robot_ee_) {
//    contact_schedule.push_back(std::make_shared<ContactSchedule>(ee,motion_parameters_->GetTotalTime(),motion_parameters_->GetContactSchedule()));
    contact_schedule.push_back(std::make_shared<ContactSchedule>(ee,motion_parameters_->contact_timings_));
    opt_variables_->AddComponent(contact_schedule.at(ee));
  }


  for (auto ee : motion_parameters_->robot_ee_) {
    // cubic spline for ee_motion
    NodeValues::Node intial_pos;
    intial_pos.at(kPos) = initial_ee_W_.At(ee);
    intial_pos.at(kVel) = Vector3d::Zero();
    auto nodes_motion = std::make_shared<EEMotionNodes>(intial_pos, contact_schedule.at(ee), motion_parameters_->ee_splines_per_swing_phase_, ee);
    opt_variables_->AddComponent(nodes_motion);
  }

// Endeffector Forces
  for (auto ee : motion_parameters_->robot_ee_) {
    // cubic spline for ee_forces
    NodeValues::Node intial_force;
    intial_force.at(kPos) = Vector3d::Zero();
    intial_force.at(kPos).z() = motion_parameters_->GetAvgZForce();
    intial_force.at(kVel) = Vector3d::Zero();
    auto nodes_forces = std::make_shared<EEForcesNodes>(intial_force, contact_schedule.at(ee), motion_parameters_->force_splines_per_stance_phase_, ee);
    opt_variables_->AddComponent(nodes_forces);
  }


  // BASE_MOTION
  std::vector<double> base_spline_timings_ = motion_parameters_->GetBasePolyDurations();

//  NodeValues::Node n;
//  n.at(kPos) = inital_base_.lin.p_;
//  n.at(kVel) = Vector3d::Zero();
//  auto spline_lin = std::make_shared<NodeValues>();
//  spline_lin->Init(n, base_spline_timings_, id::base_linear);
//  opt_variables_->AddComponent(spline_lin);
//
//  n.at(kPos) = inital_base_.ang.p_;
//  auto spline_ang = std::make_shared<NodeValues>();
//  spline_ang->Init(n, base_spline_timings_, id::base_angular);
//  opt_variables_->AddComponent(spline_ang);


  int order = motion_parameters_->order_coeff_polys_;
  int n_dim = inital_base_.lin.kNumDim;

  for (int i=0; i<base_spline_timings_.size(); ++i) {
    auto p_lin = std::make_shared<Polynomial>(order, n_dim);
    p_lin->SetConstantPos(inital_base_.lin.p_);
    opt_variables_->AddComponent(std::make_shared<PolynomialVars>(id::base_linear+std::to_string(i), p_lin));
  }

  for (int i=0; i<base_spline_timings_.size(); ++i) {
    auto p_ang = std::make_shared<Polynomial>(order, n_dim);
    p_ang->SetConstantPos(inital_base_.ang.p_);
    opt_variables_->AddComponent(std::make_shared<PolynomialVars>(id::base_angular+std::to_string(i), p_ang));
  }



  opt_variables_->Print();
}

void
MotionOptimizerFacade::SolveProblem (NlpSolver solver)
{
  BuildVariables();

  CostConstraintFactory factory;
  factory.Init(opt_variables_, motion_parameters_,
               initial_ee_W_, inital_base_, final_base_);

  nlp.Init(opt_variables_);

  auto constraints = std::make_unique<Composite>("constraints", true);
  for (ConstraintName name : motion_parameters_->GetUsedConstraints()) {
    constraints->AddComponent(factory.GetConstraint(name));
  }
  constraints->Print();
  nlp.AddConstraint(std::move(constraints));


  auto costs = std::make_unique<Composite>("costs", false);
  for (const auto& pair : motion_parameters_->GetCostWeights()) {
    CostName name = pair.first;
    costs->AddComponent(factory.GetCost(name));
  }
  costs->Print();
  nlp.AddCost(std::move(costs));


  switch (solver) {
    case Ipopt:   IpoptAdapter::Solve(nlp); break;
    case Snopt:   SnoptAdapter::Solve(nlp); break;
    default: assert(false); // solver not implemented
  }

  opt_variables_->Print();
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt) const
{
  RobotStateVec trajectory;

  auto base_lin = Spline::BuildSpline(opt_variables_, id::base_linear, motion_parameters_->GetBasePolyDurations());
  auto base_ang = Spline::BuildSpline(opt_variables_, id::base_angular, motion_parameters_->GetBasePolyDurations());


  using SplineT = std::shared_ptr<Spline>;
  std::vector<SplineT> ee_splines;
  std::vector<SplineT> ee_forces_spline;
  std::vector<std::shared_ptr<ContactSchedule>> contact_schedules;
  int n_ee = motion_parameters_->GetEECount();
  for (int ee=0; ee<n_ee; ++ee) {

    auto contact_schedule = std::dynamic_pointer_cast<ContactSchedule>(opt_variables_->GetComponent(id::GetEEScheduleId(ee)));
    contact_schedules.push_back(contact_schedule);

    auto ee_spline = Spline::BuildSpline(opt_variables_, id::GetEEId(ee), {});
    ee_splines.push_back(ee_spline);

    auto force_spline = Spline::BuildSpline(opt_variables_, id::GetEEForceId(ee), {});
    ee_forces_spline.push_back(force_spline);
  }



  double t=0.0;
  double T = motion_parameters_->GetTotalTime();
  while (t<=T+1e-5) {

    RobotStateCartesian state(n_ee);

    State3d base; // positions and orientations set to zero
    base.lin = base_lin->GetPoint(t); //inital_base_.lin;
    base.ang = AngularStateConverter::GetState(base_ang->GetPoint(t));
    state.SetBase(base);


    RobotStateCartesian::FeetArray ee_state(n_ee);
    RobotStateCartesian::ContactState contact_state(n_ee);
    Endeffectors<Vector3d> ee_force_array(n_ee);
    for (auto ee : state.GetEndeffectors()) {
      contact_state.At(ee)  = contact_schedules.at(ee)->IsInContact(t);
      ee_state.At(ee)       = ee_splines.at(ee)->GetPoint(t);
      ee_force_array.At(ee) = ee_forces_spline.at(ee)->GetPoint(t).p_;
    }

    state.SetEEStateInWorld(ee_state);
    state.SetEEForcesInWorld(ee_force_array);
    state.SetContactState(contact_state);

    state.SetTime(t);
    trajectory.push_back(state);
    t += dt;
  }

  return trajectory;
}

void
MotionOptimizerFacade::SetMotionParameters (const MotionParametersPtr& params)
{
  motion_parameters_ = params;
}


} /* namespace opt */
} /* namespace xpp */

