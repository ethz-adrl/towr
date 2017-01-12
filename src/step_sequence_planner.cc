/**
 @file    step_sequence_planner.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Brief description
 */

#include <xpp/opt/step_sequence_planner.h>

namespace xpp {
namespace opt {

using namespace xpp::utils; // X,Y

StepSequencePlanner::StepSequencePlanner ()
{
//  robot_ = HyqRobotInterface();
}

StepSequencePlanner::~StepSequencePlanner ()
{
  // TODO Auto-generated destructor stub
}

void
StepSequencePlanner::Init (const State& curr, const State& goal,
                           const StartStance& start_stance, double robot_height,
                           int curr_swingleg)
{
  curr_state_ = curr;
  goal_state_ = goal;
  start_stance_ = start_stance;
  robot_height_ = robot_height;
  curr_swingleg_ = curr_swingleg;
}

StepSequencePlanner::AllPhaseSwingLegs
StepSequencePlanner::DetermineStepSequence (const MotionTypePtr& motion_type)
{
  // based on distance to cover
  const double width_per_step = 0.13;
  Eigen::Vector2d start_to_goal = goal_state_.p.topRows(kDim2d) - curr_state_.p.topRows(kDim2d);
  int req_steps_by_length = std::ceil(std::fabs(start_to_goal.x())/motion_type->max_step_length_);
  int req_steps_by_width  = std::ceil(std::fabs(start_to_goal.y())/width_per_step);
  int req_steps_per_leg = std::max(req_steps_by_length,req_steps_by_width);

  bool moving_mainly_in_x = std::fabs(start_to_goal.x()) > std::fabs(0.5*start_to_goal.y());
  bool walking_back       = goal_state_.p.x() <= curr_state_.p.x();
  bool walking_right      = goal_state_.p.y() <= curr_state_.p.y();

  AllPhaseSwingLegs step_sequence;
  for (int cycle = 0; cycle<req_steps_per_leg; ++cycle)
    for (auto phase : motion_type->GetOneCycle())
      step_sequence.push_back(phase);

  // reserve walking order if walking backwards or to the right
  if (motion_type->id_ == opt::WalkID)
    if ((moving_mainly_in_x && walking_back) || (!moving_mainly_in_x && walking_right))
      std::reverse(step_sequence.begin(),step_sequence.end());

  return step_sequence;
}

} /* namespace opt */
} /* namespace xpp */

