/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/optimization_variables.h>
#include <xpp/opt/motion_optimizer_facade.h>
#include <xpp/opt/com_motion.h>
//#include <xpp/opt/motion_structure.h>
//#include <xpp/opt/wb_traj_generator.h>

#include <xpp/opt/endeffectors_motion.h>

#include <xpp/opt/motion_structure.h>

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
//
//
//  MotionStructure motion_structure;
//  motion_structure.Init(motion_type_->robot_ee_,
//                        start_geom_.GetEEPos(),
//                        phase_specs,
//                        start_geom_.GetPercentPhase(),
//                        motion_type_->dt_nodes_ );
//  motion_phases_ = motion_structure.GetPhases();

  auto goal_com = goal_geom_;
  goal_com.p += motion_type_->offset_geom_to_com_;


  // initialize the ee_motion with the fixed parameters
  ee_motion_ = std::make_shared<EndeffectorsMotion>();
  ee_motion_->SetInitialPos(start_geom_.GetEEPos());
  // zmp_ maybe start by just taking two trott 4 steps and hardcode
  ee_motion_->Set2StepTrott();











  nlp_facade_.BuildNlp(start_geom_.GetBase().lin.Get2D(),
                       goal_com.Get2D(),
                       ee_motion_,
                       motion_type_);

  nlp_facade_.SolveNlp(solver);
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt)
{
//  // zmp_ this should implicitly be in the com and endeffector motion
//  // so remove entire class
//  WBTrajGenerator wb_traj_generator;
//  wb_traj_generator.Init(motion_phases_,
//                         nlp_facade_.GetComMotion(),
//                         ee_motion_->GetAllFreeContacts(),
//                         ee_motion_,
//                         start_geom_,
//                         motion_type_->offset_geom_to_com_);
//
//  return wb_traj_generator.BuildWholeBodyTrajectory(dt);
//
  RobotStateVec trajectory;

  double t=0.0;
  double T = motion_type_->GetTotalTime();
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
//  start_geom_.SetPercentPhase(0.0);
  start_geom_.SetEEState(feet_zero_z_W);
}

//MotionOptimizerFacade::ContactVec
//MotionOptimizerFacade::GetContactVec ()
//{
//  return nlp_facade_.GetContacts();
//}

void
MotionOptimizerFacade::SetMotionType (const MotionTypePtr& motion_type)
{
  motion_type_ = motion_type;
//  motion_type_ = MotionParameters::MakeMotion(id);
}

} /* namespace opt */
} /* namespace xpp */


