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

namespace xpp {
namespace opt {

using MotionStructure = xpp::opt::MotionStructure;

MotionOptimizerFacade::MotionOptimizerFacade ()
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
  // create the fixed motion structure
  step_sequence_planner_.Init(curr_state_.base_.lin.Get2D(), goal_cog_.Get2D(),
                              curr_state_.GetStanceLegsInWorld(),
                              motion_type_->walking_height_,
                              curr_state_.SwinglegID());

  auto phase_swinglegs = step_sequence_planner_.DetermineStepSequence(motion_type_);
  bool insert_final_stance = true;

  MotionStructure motion_structure;
  motion_structure.Init(curr_state_.GetStanceLegsInWorld(), phase_swinglegs,
                        motion_type_->t_phase_, motion_type_->t_phase_,
                        motion_type_->start_with_stance_, insert_final_stance, motion_type_->dt_nodes_ );

  nlp_facade_.BuildNlp(curr_state_.base_.lin.Get2D(),
                       goal_cog_.Get2D(),
                       motion_structure,
                       motion_type_);

  nlp_facade_.SolveNlp(solver);
  motion_phases_ = motion_structure.GetPhases();
  nlp_facade_.VisualizeSolution();
}

MotionOptimizerFacade::HyqStateVec
MotionOptimizerFacade::GetTrajectory (double dt) const
{
  WBTrajGenerator wb_traj_gen4_;
  double percent_swing = 0.0; // zmp_ make this part of current state
  wb_traj_gen4_.Init(motion_phases_,
                     nlp_facade_.GetComMotion(),
                     nlp_facade_.GetFootholds(),
                     motion_type_->walking_height_,
                     curr_state_.ConvertToCartesian(),
                     motion_type_->lift_height_,
                     percent_swing);

  auto art_rob_vec = wb_traj_gen4_.BuildWholeBodyTrajectory(dt);
  return curr_state_.BuildWholeBodyTrajectory(art_rob_vec);
}

void
MotionOptimizerFacade::SetCurrent (const HyqState& curr)
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


