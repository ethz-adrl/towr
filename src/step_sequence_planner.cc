/**
 @file    step_sequence_planner.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Brief description
 */

#include <xpp/hyq/step_sequence_planner.h>
#include <xpp/hyq/support_polygon.h>
#include <xpp/zmp/zmp_constraint_builder.h>
#include <cassert>

namespace xpp {
namespace hyq {

StepSequencePlanner::StepSequencePlanner ()
{
  // must still initialize current and goal state
  prev_swing_leg_ = RF;
}

StepSequencePlanner::~StepSequencePlanner ()
{
  // TODO Auto-generated destructor stub
}

void
StepSequencePlanner::SetCurrAndGoal (const State& curr, const State& goal)
{
  curr_state_ = curr;
  goal_state_ = goal;
}

StepSequencePlanner::LegIDVec
StepSequencePlanner::DetermineStepSequence (int curr_swing_leg)
{
  if (!IsStepNecessary()) {
    return std::vector<xpp::hyq::LegID>(); // empty vector, take no steps
  } else {

    //    // based on distance to cover
    //  const double length_per_step = 0.25;
    //  const double width_per_step = 0.15;
    //  Eigen::Vector2d start_to_goal = goal_state_.p.segment<2>(0) - curr_state_.p.segment<2>(0);
    //    int req_steps_by_length = std::ceil(std::fabs(start_to_goal.x())/length_per_step);
    //    int req_steps_by_width  = std::ceil(std::fabs(start_to_goal.y())/width_per_step);
    //    int req_steps_per_leg = std::max(req_steps_by_length,req_steps_by_width);
    // hardcoded 4 steps
    int req_steps_per_leg = 1;

    LegID sl;
    if (curr_swing_leg == hyq::NO_SWING_LEG)
      sl = prev_swing_leg_;
    else {
      sl = static_cast<LegID>(curr_swing_leg);
      prev_swing_leg_ = sl;
    }


    LegIDVec step_sequence;

    for (int step=0; step<4/*req_steps_per_leg*4*/; ++step) {
      step_sequence.push_back(NextSwingLeg(sl));
      sl = step_sequence.back();
    }

    return step_sequence;
  }
}

bool
StepSequencePlanner::StartWithStancePhase (const VecFoothold& start_stance,
                                           double robot_height,
                                           const LegIDVec& step_sequence) const
{
  bool start_with_stance_phase = false;

  if (step_sequence.empty())
    start_with_stance_phase = true;
  else {
    bool zmp_inside = IsZmpInsideFirstStep(start_stance, robot_height, step_sequence.front());

    // so 4ls-phase not always inserted b/c of short time zmp constraints are ignored
    // when switching between disjoint support triangles.
    if (/* !zmp_inside  && */ curr_state_.v.norm() < 0.01)
      start_with_stance_phase = true;
  }

  return start_with_stance_phase;
}



bool
StepSequencePlanner::IsStepNecessary () const
{
  static const double min_distance_to_step = 0.10;//m
  Eigen::Vector2d start_to_goal = goal_state_.p.segment<2>(0) - curr_state_.p.segment<2>(0);

  bool step_necessary = (start_to_goal.norm() > min_distance_to_step)
                     /*|| (curr_state_.v.norm() > 0.1)  */;

  return step_necessary;
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

bool
StepSequencePlanner::IsZmpInsideFirstStep (const VecFoothold& start_stance,
                                           double robot_height,
                                           LegID first_step) const
{
  Eigen::Vector2d zmp = xpp::zmp::ZeroMomentPoint::CalcZmp(curr_state_.Make3D(), robot_height);

  // remove first swingleg from current stance
  VecFoothold first_stance = start_stance;
  int idx_swingleg = Foothold::GetLastIndex(first_step, first_stance);
  first_stance.erase(first_stance.begin() + idx_swingleg);

  // fixme zero margins, since i actually allow violation of zmp constraint
  // don't want 4ls to be inserted there
  MarginValues margins = hyq::SupportPolygon::GetDefaultMargins();
//  margins.at(DIAG)/=2.;
  hyq::SupportPolygon supp(first_stance, margins);

  return zmp::ZmpConstraintBuilder::IsZmpInsideSuppPolygon(zmp,supp);
}

} /* namespace hyq */
} /* namespace xpp */


