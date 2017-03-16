/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/optimization_variables.h>
#include <xpp/opt/motion_optimizer_facade.h>
#include <xpp/opt/com_motion.h>
#include <xpp/opt/endeffectors_motion.h>


namespace xpp {
namespace opt {

MotionOptimizerFacade::MotionOptimizerFacade ()
{
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
  // TODO Auto-generated destructor stub
}

void
MotionOptimizerFacade::OptimizeMotion (NlpSolver solver)
{
//  int curr_phase = start_geom_.GetCurrentPhase();
//  int phases_in_cycle = motion_type_->GetOneCycle().size();
//  int phase = curr_phase%phases_in_cycle;
//
//  MotionParameters::SwinglegPhaseVec phase_specs;
//  phase_specs.push_back({{}, 0.4}); // initial 4-leg support phase
//  for (int i = 0; i<motion_type_->opt_horizon_in_phases_; ++i) {
//
//    phase_specs.push_back(motion_type_->GetOneCycle().at(phase));
//
//    phase++;
//    if (phase == phases_in_cycle)
//      phase = 0;
//  }
//  phase_specs.push_back({{}, 0.8}); // empty vector = final four leg support phase also for contacts at end


  auto goal_com = goal_geom_;
  goal_com.p += motion_parameters_->offset_geom_to_com_;

  // initialize the ee_motion with the fixed parameters
  ee_motion_ = std::make_shared<EndeffectorsMotion>();
  ee_motion_->SetInitialPos(start_geom_.GetEEPos());
  // zmp_ maybe start by just taking two trott 4 steps and hardcode
  ee_motion_->Set2StepTrott();

  nlp_facade_.BuildNlp(start_geom_.GetBase().lin.Get2D(),
                       goal_com.Get2D(),
                       ee_motion_,
                       motion_parameters_);

  nlp_facade_.SolveNlp(solver);
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt)
{
  RobotStateVec trajectory;

  double t=0.0;
  double T = motion_parameters_->GetTotalTime();
  while (t<T) {

    RobotStateCartesian state(start_geom_.GetEEPos().GetEECount());
    state.SetBase(nlp_facade_.GetComMotion()->GetBase(t));
    state.SetEEState(ee_motion_->GetEndeffectors(t));
    state.SetContactState(ee_motion_->GetContactState(t));
    state.SetTime(t);

    trajectory.push_back(state);
    t += dt;
  }

  return trajectory;
}


void
MotionOptimizerFacade::BuildOptimizationStartState (const RobotStateCartesian& curr)
{
  auto feet_zero_z_W = curr.GetEEState();
  for (auto ee : feet_zero_z_W.GetEEsOrdered())
    feet_zero_z_W.At(ee).p.z() = 0.0;

  start_geom_ = curr;
  start_geom_.SetEEState(feet_zero_z_W);
}

void
MotionOptimizerFacade::SetMotionParameters (const MotionParametersPtr& motion_type)
{
  motion_parameters_ = motion_type;
}

} /* namespace opt */
} /* namespace xpp */


