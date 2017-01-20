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
    :curr_state_(hyq::kNumEndeffectors) // zmp_ remove this
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




  // get current phase
  int current_phase = 0;
  double t_global;
  for (int i=0; i<motion_phases_.size(); ++i) {
    t_global += motion_phases_.at(i).duration_;
    if (t_global > curr_state_.GetTime()) {
      current_phase = i; break;
    }
  }


  int n_phases_per_cycle = motion_type_->GetOneCycle().size();








  using SwingLegsInPhase  = std::vector<EndeffectorID>;
  using AllPhaseSwingLegs = std::vector<SwingLegsInPhase>;
  int n_cycles = 2;
  AllPhaseSwingLegs step_sequence;
//  step_sequence.push_back({}); // empty vector = initial four leg support phase
  for (int cycle = 0; cycle<n_cycles; ++cycle)
    for (auto phase : motion_type_->GetOneCycle())
      step_sequence.push_back(phase);

  step_sequence.push_back({}); // empty vector = final four leg support phase also for contacts at end



//  step_sequence.erase (step_sequence.begin(),step_sequence.begin()+current_phase);



  // zmp_ this must be adapted somehow
  MotionStructure motion_structure;
  motion_structure.Init(curr_state_.GetContacts(), step_sequence,
                        motion_type_->t_phase_,
                        curr_state_.GetPercentPhase(),
                        motion_type_->dt_nodes_ );
  motion_phases_ = motion_structure.GetPhases();

  // strips away completed phase since time has already passed
//  UpdateMotionPhases(curr_state_.t_global_);

  nlp_facade_.BuildNlp(curr_state_.GetBase().lin.Get2D(),
                       goal_cog_.Get2D(),
                       motion_structure,
                       motion_type_);

  nlp_facade_.SolveNlp(solver);
  nlp_facade_.VisualizeSolution();
}

void
MotionOptimizerFacade::UpdateMotionPhases (double t_elapsed)
{
  // get current phase
  int current_phase = 0;
  double t_global;
  for (int i=0; i<motion_phases_.size(); ++i) {
    t_global += motion_phases_.at(i).duration_;
    if (t_global > t_elapsed) {
      current_phase = i; break;
    }
  }

  // remove all phases up to current phase
  motion_phases_.erase (motion_phases_.begin(),motion_phases_.begin()+current_phase);

  // reduce duration of now first phase
//  motion_phases_.front().duration_ *= (1-curr_state_.percent_phase_);

  // change contacts? zmp_
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt) const
{
  WBTrajGenerator wb_traj_gen4_;
  wb_traj_gen4_.Init(motion_phases_,
                     nlp_facade_.GetComMotion(),
                     nlp_facade_.GetFootholds(),
                     motion_type_->walking_height_,
                     curr_state_,
                     motion_type_->lift_height_);

  return wb_traj_gen4_.BuildWholeBodyTrajectory(dt);
}

void
MotionOptimizerFacade::SetCurrent (const RobotState& curr)
{
  curr_state_ = curr;
}

void
MotionOptimizerFacade::SetMotionType (MotionTypeID id)
{
  motion_type_ = MotionParameters::MakeMotion(id);
}

} /* namespace opt */
} /* namespace xpp */


