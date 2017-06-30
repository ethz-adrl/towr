/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/opt/motion_optimizer_facade.h>

#include <Eigen/Dense>
#include <kindr/Core>

#include <xpp/endeffectors.h>

#include <xpp/ipopt_adapter.h>
#include <xpp/opt/angular_state_converter.h>
#include <xpp/opt/cost_constraint_factory.h>
#include <xpp/opt/polynomial_spline.h>
#include <xpp/opt/variables/contact_schedule.h>
#include <xpp/opt/variables/endeffectors_force.h>
#include <xpp/opt/variables/endeffectors_motion.h>
#include <xpp/opt/variables/variable_names.h>
#include <xpp/snopt_adapter.h>

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

  // zmp_ remove these
  // initialize the ee_motion with the fixed parameters
  auto ee_motion = std::make_shared<EndeffectorsMotion>(initial_ee_W_,
                                                        *contact_schedule);
  opt_variables_->AddComponent(ee_motion);

  for (auto ee : motion_parameters_->robot_ee_) {
    auto ee_poly = std::make_shared<PolynomialSpline>(id::endeffectors_motion+std::to_string(ee));
    ee_poly->Init(contact_schedule->GetTimePerPhase(ee), initial_ee_W_.At(ee));
    opt_variables_->AddComponent(ee_poly);
  }


  double T = motion_parameters_->GetTotalTime();
  auto base_linear = std::make_shared<PolynomialSpline>(id::base_linear);
  base_linear->Init(T, motion_parameters_->duration_polynomial_,
                    inital_base_.lin.p_);
  opt_variables_->AddComponent(base_linear);



  // Represent roll, pitch and yaw by a spline each.
  // These angles are to be interpreted as mapping a vector expressed in
  // base frame to world frame.
  auto base_angular = std::make_shared<PolynomialSpline>(id::base_angular);
  base_angular->Init(T, motion_parameters_->duration_polynomial_,
                     inital_base_.ang.p_);
  opt_variables_->AddComponent(base_angular);



  auto force = std::make_shared<EndeffectorsForce>(motion_parameters_->load_dt_,
                                                   *contact_schedule);
  opt_variables_->AddComponent(force);

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
  auto ee_motion        = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_variables_->GetComponent(id::endeffectors_motion));
  auto contact_schedule = std::dynamic_pointer_cast<ContactSchedule>   (opt_variables_->GetComponent(id::contact_schedule));
  auto ee_forces        = std::dynamic_pointer_cast<EndeffectorsForce> (opt_variables_->GetComponent(id::endeffector_force));

  double t=0.0;
  double T = motion_parameters_->GetTotalTime();
  while (t<=T+1e-5) {

    RobotStateCartesian state(initial_ee_W_.GetCount());

    State3d base; // positions and orientations set to zero
    base.lin = base_lin->GetPoint(t);
    base.ang = AngularStateConverter::GetState(base_ang->GetPoint(t));
    state.SetBase(base);

    state.SetEEStateInWorld(ee_motion->GetEndeffectors(t));
    state.SetEEForcesInWorld(ee_forces->GetForce(t));
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

void
MotionOptimizerFacade::SetInitialState (const RobotStateCartesian& initial_state)
{
  initial_ee_W_       = initial_state.GetEEPos();

  inital_base_ = State3dEuler(); // zero
  inital_base_.lin = initial_state.GetBase().lin;

  kindr::RotationQuaternionD quat(initial_state.GetBase().ang.q);
  kindr::EulerAnglesZyxD euler(quat);
  euler.setUnique(); // to express euler angles close to 0,0,0, not 180,180,180 (although same orientation)
  inital_base_.ang.p_ = euler.toImplementation().reverse();
  // assume zero euler rates and euler accelerations
}


} /* namespace opt */
} /* namespace xpp */

