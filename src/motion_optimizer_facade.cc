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
#include <xpp/opt/variables/contact_timings.h>
#include <xpp/opt/variables/polynomial_spline.h>
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


  for (auto ee : motion_parameters_->robot_ee_) {

    bool ee_initially_in_contact = contact_schedule->GetPhases(ee).front().first;

    std::string id_motion = id::endeffectors_motion+std::to_string(ee);
    auto ee_poly = std::make_shared<EndeffectorSpline>(id_motion, ee_initially_in_contact);
    ee_poly->Init(contact_schedule->GetTimePerPhase(ee),
                                     4,
                                     initial_ee_W_.At(ee));
    opt_variables_->AddComponent(ee_poly);

    std::string id_force  = id::endeffector_force+std::to_string(ee);
    auto ee_force = std::make_shared<ForceSpline>(id_force,
                                                  ee_initially_in_contact,
                                                  motion_parameters_->GetForceLimit());
    double fz_stand = motion_parameters_->GetMass()*kGravity/motion_parameters_->GetEECount();
    ee_force->Init(contact_schedule->GetTimePerPhase(ee),
                                      motion_parameters_->polys_per_ee_phase_,
                                      3,
                                      Vector3d(0.0, 0.0, fz_stand));
    opt_variables_->AddComponent(ee_force);


    // add timings as optimization variables
    int max_n_steps = 3; // zmp_ move to parameters
    auto ee_timings = std::make_shared<ContactTimings>(ee, max_n_steps);
    opt_variables_->AddComponent(ee_timings);
  }


  double T = motion_parameters_->GetTotalTime();
  auto base_linear = std::make_shared<PolynomialSpline>(id::base_linear);
  base_linear->Init(T, motion_parameters_->duration_polynomial_,
                                       4,inital_base_.lin.p_);
  opt_variables_->AddComponent(base_linear);


  // Represent roll, pitch and yaw by a spline each.
  // These angles are to be interpreted as mapping a vector expressed in
  // base frame to world frame.
  auto base_angular = std::make_shared<PolynomialSpline>(id::base_angular);
  base_angular->Init(T, motion_parameters_->duration_polynomial_,
                                        4,inital_base_.ang.p_);
  opt_variables_->AddComponent(base_angular);

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

  auto base_lin         = std::dynamic_pointer_cast<PolynomialSpline>  (opt_variables_->GetComponent(id::base_linear));
  auto base_ang         = std::dynamic_pointer_cast<PolynomialSpline>  (opt_variables_->GetComponent(id::base_angular));
  auto contact_schedule = std::dynamic_pointer_cast<ContactSchedule>   (opt_variables_->GetComponent(id::contact_schedule));


  std::vector<std::shared_ptr<PolynomialSpline>> ee_splines;
  std::vector<std::shared_ptr<PolynomialSpline>> ee_forces_spline;
  int n_ee = motion_parameters_->GetEECount();
  for (int i=0; i<n_ee; ++i) {
    std::string id_motion = id::endeffectors_motion+std::to_string(i);
    ee_splines.push_back(std::dynamic_pointer_cast<PolynomialSpline>(opt_variables_->GetComponent(id_motion)));

    std::string id_force = id::endeffector_force+std::to_string(i);
    ee_forces_spline.push_back(std::dynamic_pointer_cast<PolynomialSpline>(opt_variables_->GetComponent(id_force)));
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
      ee_state.At(ee) = ee_splines.at(ee)->GetPoint(t);
      ee_force_array.At(ee) = ee_forces_spline.at(ee)->GetPoint(t).p_;
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

