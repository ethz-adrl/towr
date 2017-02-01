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
  int curr_phase = opt_start_state_.GetCurrentPhase();
  int phases_in_cycle = motion_type_->GetOneCycle().size();
  int phase = curr_phase%phases_in_cycle;

  using SwingLegsInPhase  = std::vector<EndeffectorID>;
  using AllPhaseSwingLegs = std::vector<SwingLegsInPhase>;
  AllPhaseSwingLegs step_sequence;
  for (int i = 0; i<motion_type_->opt_horizon_in_phases_; ++i) {

    step_sequence.push_back(motion_type_->GetOneCycle().at(phase));

    phase++;
    if (phase == phases_in_cycle)
      phase = 0;
  }
  step_sequence.push_back({}); // empty vector = final four leg support phase also for contacts at end


  MotionStructure motion_structure;
  motion_structure.Init(opt_start_state_.GetEEPos(), step_sequence,
                        motion_type_->t_phase_,
                        opt_start_state_.GetPercentPhase(),
                        motion_type_->dt_nodes_ );
  motion_phases_ = motion_structure.GetPhases();

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
                         motion_type_->geom_walking_height_,
                         opt_start_state_,
                         motion_type_->lift_height_,
                         motion_type_->offset_geom_to_com_);

  state_second_phase_ = wb_traj_generator.GetNodeSecondPhase();

  return wb_traj_generator.BuildWholeBodyTrajectory(dt);
}

void
MotionOptimizerFacade::BuildOptimizationStartState (const RobotState& curr)
{
  // zmp_ !!!!this is HUGE! very important
  auto feet_zero_z_W = curr.GetEEState();
  for (auto ee : feet_zero_z_W.GetEEsOrdered())
    feet_zero_z_W.At(ee).p.z() = 0.0;

  static bool first_time = true;
  if (first_time) {
    state_second_phase_ = curr;
    state_second_phase_.SetEEState(feet_zero_z_W);
    first_time = false;
  }

  opt_start_state_ = curr;
  opt_start_state_.SetPercentPhase(0.0);
  opt_start_state_.SetEEState(feet_zero_z_W/*state_second_phase_.GetEEState()*/);
}

void
MotionOptimizerFacade::SetMotionType (MotionTypeID id)
{
  motion_type_ = MotionParameters::MakeMotion(id);
}

} /* namespace opt */
} /* namespace xpp */


