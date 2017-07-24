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

#include <xpp/opt/ipopt_adapter.h>
#include <xpp/opt/snopt_adapter.h>

namespace xpp {
namespace opt {

MotionOptimizerFacade::MotionOptimizerFacade ()
{
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

  // initialize the contact schedule
  auto contact_schedule = std::make_shared<ContactSchedule>(
      motion_parameters_->GetContactSchedule());
  opt_variables_->AddComponent(contact_schedule);

  int n_dim = inital_base_.lin.kNumDim;


  for (auto ee : motion_parameters_->robot_ee_) {

//    // add timings as optimization variables
//    auto ee_timings = std::make_shared<ContactTimings>(ee, contact_schedule->GetTimePerPhase(ee));
//    opt_variables_->AddComponent(ee_timings);
    auto timings = contact_schedule->GetTimePerPhase(ee);
    bool ee_initially_in_contact = contact_schedule->GetPhases(ee).front().first;
    int n_phases = timings.size();

    // use two polynomials for each endeffector phase
    int n_polys_per_phase = 2;
    std::vector<double> timings2;
    for (double t : timings)
      for (int i=0; i<n_polys_per_phase; ++i)
        timings2.push_back(t/n_polys_per_phase);



    // EE_MOTION
//    std::string id_motion = id::endeffectors_motion+std::to_string(ee);

    std::string id_force  = id::endeffector_force+std::to_string(ee);
    double fz_stand = motion_parameters_->GetMass()*kGravity/motion_parameters_->GetEECount();
    int order_poly = 4;

    for (int i=0; i<n_phases; ++i) {



//      // traditional spline
//      auto p_motion = std::make_shared<Polynomial>(order_poly, n_dim);
//      p_motion->SetConstantPos(initial_ee_W_.At(ee));
//      opt_variables_->AddComponent(std::make_shared<PolynomialVars>(id::GetEEId(ee)+std::to_string(i), p_motion));


//      auto p_force = std::make_shared<Polynomial>(order_poly, n_dim, id_force + std::to_string(i));
//      p_force->SetCoefficients(Polynomial::A, Vector3d(0.0, 0.0, fz_stand));
//      opt_variables_->AddComponent(p_force);
    }


    // try cubic spline
    NodeValues::Node n;
    n.at(kPos) = initial_ee_W_.At(ee);
    n.at(kVel) = Vector3d::Zero();
    auto node_values = std::make_shared<NodeValues>(n, timings2, id::GetEEId(ee));
    opt_variables_->AddComponent(node_values);


//    ee_poly->Init(n_phases*polys_per_duration, order_poly,initial_ee_W_.At(ee));
//    ee_poly->SetPhaseDurations(timings, polys_per_duration);
//    opt_variables_->AddComponent(ee_poly);


//    // EE_FORCES
//    auto ee_force = std::make_shared<ForceSpline>(id::GetEEForce(ee),
//                                                  ee_initially_in_contact,
//                                                  motion_parameters_->GetForceLimit());
//    double fz_stand = motion_parameters_->GetMass()*kGravity/motion_parameters_->GetEECount();
//    ee_force->Init(n_phases*motion_parameters_->polys_per_force_phase_, 3, Vector3d(0.0, 0.0, fz_stand));
//    ee_force->SetPhaseDurations(timings, motion_parameters_->polys_per_force_phase_);
//    opt_variables_->AddComponent(ee_force);

  }


  // BASE_MOTION
  std::vector<double> base_spline_timings_ = motion_parameters_->GetBasePolyDurations();

//  NodeValues::Node n;
//  n.at(kPos) = inital_base_.lin.p_;
//  n.at(kVel) = Vector3d::Zero();
//  auto spline_lin = std::make_shared<NodeValues>(n, base_spline_timings_, id::base_linear);
//  opt_variables_->AddComponent(spline_lin);
//
//  n.at(kPos) = inital_base_.ang.p_;
//  auto spline_ang = std::make_shared<NodeValues>(n, base_spline_timings_, id::base_angular);
//  opt_variables_->AddComponent(spline_ang);


  for (int i=0; i<base_spline_timings_.size(); ++i) {
    auto p_lin = std::make_shared<Polynomial>(4, n_dim);
    p_lin->SetConstantPos(inital_base_.lin.p_);
    opt_variables_->AddComponent(std::make_shared<PolynomialVars>(id::base_linear+std::to_string(i), p_lin));

    auto p_ang = std::make_shared<Polynomial>(4, n_dim);
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
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt) const
{
  RobotStateVec trajectory;

  auto base_lin = Spline::BuildSpline(opt_variables_, id::base_linear, motion_parameters_->GetBasePolyDurations());
  auto base_ang = Spline::BuildSpline(opt_variables_, id::base_angular, motion_parameters_->GetBasePolyDurations());

//  auto base_lin         = std::dynamic_pointer_cast<PolynomialSpline>  (opt_variables_->GetComponent(id::base_linear));
//  auto base_ang         = std::dynamic_pointer_cast<PolynomialSpline>  (opt_variables_->GetComponent(id::base_angular));
  auto contact_schedule = std::dynamic_pointer_cast<ContactSchedule>   (opt_variables_->GetComponent(id::contact_schedule));


  using SplineT = std::shared_ptr<Spline>;
  std::vector<SplineT> ee_splines;
  std::vector<SplineT> ee_forces_spline; // zmp_ add forces again
  int n_ee = motion_parameters_->GetEECount();
  for (int i=0; i<n_ee; ++i) {
//    std::string id_motion = id::endeffectors_motion+std::to_string(i);
    auto durations = contact_schedule->GetTimePerPhase(static_cast<EndeffectorID>(i));
    auto ee_spline = Spline::BuildSpline(opt_variables_, id::GetEEId(i), durations);
    ee_splines.push_back(ee_spline);

//    auto force_spline = PolynomialSpline::BuildSpline(opt_variables_, id::GetEEForceId(i), contact_schedule->GetTimePerPhase(static_cast<EndeffectorID>(i)));
//    ee_forces_spline.push_back(force_spline);
  }



  double t=0.0;
  double T = motion_parameters_->GetTotalTime();
  while (t<=T+1e-5) {

    RobotStateCartesian state(n_ee);

    State3d base; // positions and orientations set to zero
    base.lin = base_lin->GetPoint(t);
    base.ang = AngularStateConverter::GetState(base_ang->GetPoint(t));
    state.SetBase(base);


    RobotStateCartesian::FeetArray ee_state(n_ee);
    Endeffectors<Vector3d> ee_force_array(n_ee);
    for (auto ee : state.GetEndeffectors()) {
      ee_state.At(ee)       = ee_splines.at(ee)->GetPoint(t);
//      ee_force_array.At(ee) = ee_forces_spline.at(ee)->GetPoint(t).p_;
    }

    state.SetEEStateInWorld(ee_state);


    state.SetEEForcesInWorld(ee_force_array);
    state.SetContactState(contact_schedule->IsInContact(t));

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

