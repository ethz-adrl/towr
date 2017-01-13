/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/opt/motion_optimizer_facade.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/opt/optimization_variables.h>

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
MotionOptimizerFacade::Init (double lift_height,
                             double outward_swing,
                             double trajectory_dt,
                             VisualizerPtr visualizer)
{
  wb_traj_gen4_.SetParams(0.5, lift_height, outward_swing, trajectory_dt);
  nlp_facade_.AttachNlpObserver(visualizer);
}

void
MotionOptimizerFacade::OptimizeMotion ()
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

  nlp_facade_.SolveNlp(Snopt);

  wb_traj_gen4_.Init(motion_structure.GetPhases(),
                     nlp_facade_.GetComMotion(),
                     nlp_facade_.GetFootholds(),
                     motion_type_->walking_height_,
                     curr_state_.ConvertToCartesian());

  auto art_rob_vec = wb_traj_gen4_.BuildWholeBodyTrajectory();

  optimized_trajectory_ = curr_state_.BuildWholeBodyTrajectory(art_rob_vec);
}

MotionOptimizerFacade::HyqStateVec
MotionOptimizerFacade::GetTrajectory () const
{
  return optimized_trajectory_;
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


