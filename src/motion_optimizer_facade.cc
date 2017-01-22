/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/opt/motion_optimizer_facade.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/opt/optimization_variables.h>
#include <xpp/opt/wb_traj_generator.h>

#include <xpp/hyq/ee_hyq.h>

namespace xpp {
namespace opt {

using MotionStructure = xpp::opt::MotionStructure;

MotionOptimizerFacade::MotionOptimizerFacade ()
    :opt_start_state_(hyq::kNumEndeffectors),
     state_second_phase_(opt_start_state_)// zmp_ remove this
{
  SetMotionType(WalkID);
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
  // TODO Auto-generated destructor stub
}

void
MotionOptimizerFacade::SetVisualizer (VisualizerPtr visualizer)
{
  nlp_facade_.SetVisualizer(visualizer);
}


void
MotionOptimizerFacade::OptimizeMotion (NlpSolver solver)
{
//  // create the fixed motion structure
//  // zmp_ can't use current state here
//  step_sequence_planner_.Init(curr_state_.GetBase().lin.Get2D(), goal_cog_.Get2D());
//  auto phase_swinglegs = step_sequence_planner_.DetermineStepSequence(motion_type_);

//  // get current phase
//  int current_phase = 0;
//  double t_global;
//  for (int i=0; i<motion_phases_.size(); ++i) {
//    t_global += motion_phases_.at(i).duration_;
//    if (t_global > opt_start_state_.GetTime()) {
//      current_phase = i; break;
//    }
//  }
//
//
//  int n_phases_per_cycle = motion_type_->GetOneCycle().size();


  using SwingLegsInPhase  = std::vector<EndeffectorID>;
  using AllPhaseSwingLegs = std::vector<SwingLegsInPhase>;
  AllPhaseSwingLegs step_sequence;

//  int n_cycles = 2;
////  step_sequence.push_back({}); // empty vector = initial four leg support phase
//  for (int cycle = 0; cycle<n_cycles; ++cycle)
//    for (auto phase : motion_type_->GetOneCycle())
//      step_sequence.push_back(phase);
//
//  step_sequence.erase (step_sequence.begin(),step_sequence.begin()+current_phase);


  int curr_phase = opt_start_state_.GetCurrentPhase();


  std::cout << "curr_phase: " << curr_phase << std::endl;

  int cycle_part = curr_phase%2; // either 1 or 0
//  step_sequence.push_back(motion_type_->GetOneCycle().at(cycle_part));

  for (int cycle = 0; cycle<1; ++cycle) {
    step_sequence.push_back(motion_type_->GetOneCycle().at(cycle_part));
    step_sequence.push_back(motion_type_->GetOneCycle().at(1-cycle_part));
  }
  step_sequence.push_back({}); // empty vector = final four leg support phase also for contacts at end
//  step_sequence.push_back({}); // empty vector = final four leg support phase also for contacts at end
//  step_sequence.push_back({}); // empty vector = final four leg support phase also for contacts at end
//  step_sequence.push_back({}); // empty vector = final four leg support phase also for contacts at end
//  step_sequence.push_back({}); // empty vector = final four leg support phase also for contacts at end






  // zmp_ this must be adapted somehow
  MotionStructure motion_structure;
  motion_structure.Init(opt_start_state_.GetEEPos(), step_sequence,
                        motion_type_->t_phase_,
                        opt_start_state_.GetPercentPhase(),
                        motion_type_->dt_nodes_ );
  motion_phases_ = motion_structure.GetPhases();

  // strips away completed phase since time has already passed
//  UpdateMotionPhases(curr_state_.t_global_);

  nlp_facade_.BuildNlp(opt_start_state_.GetBase().lin.Get2D(),
                       goal_cog_.Get2D(),
                       motion_structure,
                       motion_type_);

  nlp_facade_.SolveNlp(solver);
  nlp_facade_.VisualizeSolution();
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt)
{
  WBTrajGenerator wb_traj_generator;
  wb_traj_generator.Init(motion_phases_,
                     nlp_facade_.GetComMotion(),
                     nlp_facade_.GetFootholds(),
                     motion_type_->walking_height_,
                     opt_start_state_,
                     motion_type_->lift_height_);

  state_second_phase_ = wb_traj_generator.GetNodeSecondPhase();

  return wb_traj_generator.BuildWholeBodyTrajectory(dt);
}

void
MotionOptimizerFacade::BuildOptimizationStartState (const RobotState& curr)
{
  // zmp_ !!!!this is HUGE! very important
  auto feet_zero_z_W = curr.GetEEState();
  for (auto ee : feet_zero_z_W.GetEEsOrdered()) {
    feet_zero_z_W.At(ee).p.z() = 0.0;

    std::cout << "feet: " << feet_zero_z_W.At(ee).p.transpose() << std::endl;
    std::cout << "second: " << state_second_phase_.GetEEState().At(ee).p.transpose() << std::endl;
  }


  static bool first_time = true;
  if (first_time) {
    state_second_phase_ = curr;
    state_second_phase_.SetEEState(feet_zero_z_W);
    first_time = false;
  }


  opt_start_state_ = curr;
  opt_start_state_.SetPercentPhase(0.0);


  for (auto ee : feet_zero_z_W.GetEEsOrdered()) {
    std::cout << "feet: " << feet_zero_z_W.At(ee).p.transpose() << std::endl;
    std::cout << "second: " << state_second_phase_.GetEEState().At(ee).p.transpose() << std::endl;
  }



  opt_start_state_.SetEEState(feet_zero_z_W/*state_second_phase_.GetEEState()*/);

//  opt_start_state_.SetContactState(state_second_phase_.GetContactState());

//  // zmp_ so far only switching when all legs are in contact
//  RobotState::ContactState contact_state = curr.GetContactState();
//  contact_state.SetAll(true);
//  opt_start_state_.SetContactState(contact_state);
}

void
MotionOptimizerFacade::SetMotionType (MotionTypeID id)
{
  motion_type_ = MotionParameters::MakeMotion(id);
}

} /* namespace opt */
} /* namespace xpp */


