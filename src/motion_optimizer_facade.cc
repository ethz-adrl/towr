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
#include <xpp/opt/motion_parameter_instances.h>

#include <xpp/opt/ipopt_adapter.h>
#include <xpp/opt/snopt_adapter.h>

namespace xpp {
namespace opt {

MotionOptimizerFacade::MotionOptimizerFacade ()
{
  params_ = std::make_shared<HyQMotionParameters>();
  BuildDefaultInitialState();
  opt_variables_ = std::make_shared<Composite>("nlp_variables", true);
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
}

void
MotionOptimizerFacade::BuildDefaultInitialState ()
{
  auto p_nom_B = params_->GetNominalStanceInBase();

  inital_base_.lin.p_ << 0.0, 0.0, -p_nom_B.At(E0).z();
  inital_base_.ang.p_ << 0.0, 0.0, 0.0; // euler (roll, pitch, yaw)

  initial_ee_W_.SetCount(params_->GetEECount());
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {
    initial_ee_W_.At(ee) = p_nom_B.At(ee) + inital_base_.lin.p_;
    initial_ee_W_.At(ee).z() = 0.0;
  }
}

void
MotionOptimizerFacade::BuildVariables ()
{
  opt_variables_->ClearComponents();


  std::vector<std::shared_ptr<ContactSchedule>> contact_schedule;
  for (auto ee : params_->robot_ee_) {
    contact_schedule.push_back(std::make_shared<ContactSchedule>(ee,
                                                                 params_->contact_timings_.at(ee),
                                                                 params_->min_phase_duration_,
                                                                 params_->max_phase_duration_));
    opt_variables_->AddComponent(contact_schedule.at(ee));
  }


  for (auto ee : params_->robot_ee_) {
    auto nodes_motion = std::make_shared<PhaseNodes>(kDim3d,
                                                     contact_schedule.at(ee)->GetContactSequence(),
                                                     id::GetEEMotionId(ee),
                                                     params_->ee_splines_per_swing_phase_,
                                                     PhaseNodes::Motion);

    Vector3d final_ee_pos_W = final_base_.lin.p_ + params_->GetNominalStanceInBase().At(ee);
    nodes_motion->InitializeVariables(initial_ee_W_.At(ee), final_ee_pos_W, contact_schedule.at(ee)->GetTimePerPhase());
    nodes_motion->AddStartBound(kPos, initial_ee_W_.At(ee).topRows<kDim2d>()); // only xy, z given by terrain
    nodes_motion->AddFinalBound(kPos, final_ee_pos_W.topRows<kDim2d>());       // only xy, z given by terrain

    opt_variables_->AddComponent(nodes_motion);
    contact_schedule.at(ee)->AddObserver(nodes_motion);
  }

// Endeffector Forces
  for (auto ee : params_->robot_ee_) {
    auto nodes_forces = std::make_shared<PhaseNodes>(kDim3d,
                                                     contact_schedule.at(ee)->GetContactSequence(),
                                                     id::GetEEForceId(ee),
                                                     params_->force_splines_per_stance_phase_,
                                                     PhaseNodes::Force);

    Vector3d f_stance(0.0, 0.0, params_->GetAvgZForce());
    nodes_forces->InitializeVariables(f_stance, f_stance, contact_schedule.at(ee)->GetTimePerPhase());


    opt_variables_->AddComponent(nodes_forces);
    contact_schedule.at(ee)->AddObserver(nodes_forces);
  }


  // BASE_MOTION
  std::vector<double> base_spline_timings_ = params_->GetBasePolyDurations();

  auto linear  = std::make_tuple(id::base_linear,  inital_base_.lin, final_base_.lin);
  auto angular = std::make_tuple(id::base_angular, inital_base_.ang, final_base_.ang);

  for (auto tuple : {linear, angular}) {
    std::string id   = std::get<0>(tuple);
    StateLin3d init  = std::get<1>(tuple);
    StateLin3d final = std::get<2>(tuple);

    auto spline = std::make_shared<NodeValues>(init.kNumDim,  base_spline_timings_.size(), id);
    spline->InitializeVariables(init.p_, final.p_, base_spline_timings_);

    spline->AddStartBound(kPos, init.p_);
    spline->AddStartBound(kVel, init.v_);
    spline->AddFinalBound(kVel, final.v_);

    if (id == id::base_linear)
      spline->AddFinalBound(kPos, final.p_.topRows<kDim2d>()); // only xy, z given by terrain
    if (id == id::base_angular)
      spline->AddFinalBound(kPos, final.p_); // roll, pitch, yaw bound



//    // try to force jump
//    if (id == id::base_linear) {
//      Vector3d inter = (init.p_ + final.p_)/2.;
//      inter.z() = 0.9;
//      spline->AddIntermediateBound(kPos, inter);
//    }
//
//    if (id == id::base_angular) {
//      spline->AddIntermediateBound(kPos, Vector3d::Zero());
//    }



    opt_variables_->AddComponent(spline);
  }


//  int n_dim = inital_base_.lin.kNumDim;
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



//  int order = params_->order_coeff_polys_;

//
//  for (int i=0; i<base_spline_timings_.size(); ++i) {
//    auto p_lin = std::make_shared<Polynomial>(order, n_dim);
//    p_lin->SetConstantPos(inital_base_.lin.p_);
//    opt_variables_->AddComponent(std::make_shared<PolynomialVars>(id::base_linear+std::to_string(i), p_lin));
//  }
//
//  for (int i=0; i<base_spline_timings_.size(); ++i) {
//    auto p_ang = std::make_shared<Polynomial>(order, n_dim);
//    p_ang->SetConstantPos(inital_base_.ang.p_);
//    opt_variables_->AddComponent(std::make_shared<PolynomialVars>(id::base_angular+std::to_string(i), p_ang));
//  }



  opt_variables_->Print();
}

void
MotionOptimizerFacade::SolveProblem (NlpSolver solver)
{
  BuildVariables();

  CostConstraintFactory factory;
  factory.Init(opt_variables_, params_,
               initial_ee_W_, inital_base_, final_base_);

  nlp.Init(opt_variables_);

  auto constraints = std::make_unique<Composite>("constraints", true);
  for (ConstraintName name : params_->GetUsedConstraints())
    constraints->AddComponent(factory.GetConstraint(name));

  constraints->Print();
  nlp.AddConstraint(std::move(constraints));


  auto costs = std::make_unique<Composite>("costs", false);
  for (const auto& pair : params_->GetCostWeights())
    costs->AddComponent(factory.GetCost(pair.first, pair.second));

  costs->Print();
  nlp.AddCost(std::move(costs));


  switch (solver) {
    case Ipopt:   IpoptAdapter::Solve(nlp); break;
    case Snopt:   SnoptAdapter::Solve(nlp); break;
    default: assert(false); // solver not implemented
  }

  nlp.PrintCurrent();
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt) const
{
  RobotStateVec trajectory;

  auto base_lin = Spline::BuildSpline(opt_variables_, id::base_linear, params_->GetBasePolyDurations());
  auto base_ang = Spline::BuildSpline(opt_variables_, id::base_angular, params_->GetBasePolyDurations());


  using SplineT = std::shared_ptr<Spline>;
  std::vector<SplineT> ee_splines;
  std::vector<SplineT> ee_forces_spline;
  std::vector<std::shared_ptr<ContactSchedule>> contact_schedules;
  int n_ee = params_->GetEECount();
  for (int ee=0; ee<n_ee; ++ee) {

    auto contact_schedule = std::dynamic_pointer_cast<ContactSchedule>(opt_variables_->GetComponent(id::GetEEScheduleId(ee)));
    contact_schedules.push_back(contact_schedule);

    auto ee_spline = Spline::BuildSpline(opt_variables_, id::GetEEMotionId(ee), {});
    ee_splines.push_back(ee_spline);

    auto force_spline = Spline::BuildSpline(opt_variables_, id::GetEEForceId(ee), {});
    ee_forces_spline.push_back(force_spline);
  }



  double t=0.0;
  double T = params_->GetTotalTime();
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


} /* namespace opt */
} /* namespace xpp */

