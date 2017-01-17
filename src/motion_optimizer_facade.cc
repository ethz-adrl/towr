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

#include <xpp/hyq/codegen/hyq_kinematics.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>
#include <xpp/hyq/joints_hyq.h>

namespace xpp {
namespace opt {

using MotionStructure = xpp::opt::MotionStructure;

MotionOptimizerFacade::MotionOptimizerFacade ()
    :curr_state_(hyq::kNumEE, hyq::kNumJointsPerLeg) // zmp_ remove this
{
  // zmp_ remove this here as well
  curr_state_.inv_kinematics_ = std::make_shared<hyq::HyqInverseKinematics>();
  curr_state_.for_kinematics_ = std::make_shared<hyq::codegen::HyQKinematics>();

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
  wb_traj_gen4_.Init(motion_phases_,
                     nlp_facade_.GetComMotion(),
                     nlp_facade_.GetFootholds(),
                     motion_type_->walking_height_,
                     curr_state_.ConvertToCartesian(),
                     motion_type_->lift_height_);

  auto art_rob_vec = wb_traj_gen4_.BuildWholeBodyTrajectory(dt);

  // !!!!zmp_ this is where the segfault is, IK not defined here
  // move to IK class possibly?
  hyq::HyqState hyq;
  return hyq.BuildWholeBodyTrajectory(art_rob_vec);
}

void
MotionOptimizerFacade::SetCurrent (const RobotState& curr)
{
  // zmp_ this obviously also doesn't belong here
  curr_state_ = curr;
  curr_state_.inv_kinematics_ = std::make_shared<hyq::HyqInverseKinematics>();
  curr_state_.for_kinematics_ = std::make_shared<hyq::codegen::HyQKinematics>();
}

void
MotionOptimizerFacade::SetMotionType (MotionTypeID id)
{
  motion_type_ = MotionParameters::MakeMotion(id);
}

} /* namespace opt */
} /* namespace xpp */


