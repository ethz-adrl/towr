/**
 @file    step_sequence_planner.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Brief description
 */

#include <xpp/hyq/step_sequence_planner.h>
#include <xpp/hyq/ee_hyq.h>

namespace xpp {
namespace hyq {

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
StepSequencePlanner::DetermineStepSequence (const MotionType& motion_type)
{
  // based on distance to cover
  const double width_per_step = 0.13;
  Eigen::Vector2d start_to_goal = goal_state_.p.topRows(kDim2d) - curr_state_.p.topRows(kDim2d);
  int req_steps_by_length = std::ceil(std::fabs(start_to_goal.x())/motion_type->max_step_length_);
  int req_steps_by_width  = std::ceil(std::fabs(start_to_goal.y())/width_per_step);
  int req_steps_per_leg = std::max(req_steps_by_length,req_steps_by_width);
  int n_steps = 4*req_steps_per_leg;

  LegID last_swingleg = RF;

  bool moving_mainly_in_x = std::fabs(start_to_goal.x()) > std::fabs(0.5*start_to_goal.y());
  bool walking_forward = goal_state_.p.x() >= curr_state_.p.x();
  bool walking_left    = goal_state_.p.y() >= curr_state_.p.y();

  int n_phases = n_steps/motion_type->swinglegs_per_phase_;

  std::vector<LegIDVec> step_sequence;
  for (int phase=0; phase<n_phases; ++phase) {

    switch (motion_type->id_) {
      case opt::TrottID: {
        LegIDVec swinglegs_lf = {LF, RH};
        LegIDVec swinglegs_rf = {RF, LH};

        if (phase%2==0)
          step_sequence.push_back(swinglegs_lf);
        else
          step_sequence.push_back(swinglegs_rf);

        break;
      }
      case opt::WalkID: {
        if (moving_mainly_in_x) {
          if (walking_forward)
            step_sequence.push_back({NextSwingLeg(last_swingleg)});
          else
            step_sequence.push_back({NextSwingLegBackwards(last_swingleg)});
        } else { // moving mainly in y
          if (walking_left)
            step_sequence.push_back({NextSwingLeg(last_swingleg)});
          else
            step_sequence.push_back({NextSwingLegBackwards(last_swingleg)});
        }
        last_swingleg = step_sequence.back().back();
        break;
      }
    }
  }

  return ConvertToEE(step_sequence);
}

LegID
StepSequencePlanner::NextSwingLeg (LegID curr) const
{
  switch (curr) {
    case LH: return LF;
    case LF: return RH;
    case RH: return RF;
    case RF: return LH;
    default: assert(false); // this should never happen
  };
}

LegID
StepSequencePlanner::NextSwingLegBackwards (LegID curr) const
{
  switch (curr) {
    case LH: return RF;
    case RF: return RH;
    case RH: return LF;
    case LF: return LH;
    default: assert(false); // this should never happen
  };
}

StepSequencePlanner::AllPhaseSwingLegs
StepSequencePlanner::ConvertToEE (const std::vector<LegIDVec>& hyq)
{
  std::vector<SwingLegsInPhase> xpp;

  for (auto hyq_swinglegs : hyq) {
    SwingLegsInPhase swinglegs_in_phase;
    for (auto leg : hyq_swinglegs)
      swinglegs_in_phase.push_back(kMapHyqToOpt.at(leg));

    xpp.push_back(swinglegs_in_phase);
  }

  return xpp;
}

} /* namespace hyq */
} /* namespace xpp */

