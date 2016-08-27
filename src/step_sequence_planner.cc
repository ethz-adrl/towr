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
StepSequencePlanner::Init (const State& curr, const State& goal,
                           const VecFoothold& start_stance, double robot_height)
{
  curr_state_ = curr;
  goal_state_ = goal;
  start_stance_ = start_stance;
  robot_height_ = robot_height;
}

StepSequencePlanner::LegIDVec
StepSequencePlanner::DetermineStepSequence (int swingleg_of_last_spline)
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

    LegID last_swingleg;
    if (swingleg_of_last_spline == hyq::NO_SWING_LEG)
      last_swingleg = prev_swing_leg_;
    else {
      last_swingleg = static_cast<LegID>(swingleg_of_last_spline);
      prev_swing_leg_ = last_swingleg;
    }


    LegIDVec step_sequence;
    int n_steps = 5; // how many steps to take

    for (int step=0; step<n_steps/*req_steps_per_leg*4*/; ++step) {
      step_sequence.push_back(NextSwingLeg(last_swingleg));
      last_swingleg = step_sequence.back();
    }

    return step_sequence;
  }
}

bool
StepSequencePlanner::IsStepNecessary () const
{
  static const double min_distance_to_step = 0.08;//m
  Vector2d start_to_goal = goal_state_.p.segment<2>(0) - curr_state_.p.segment<2>(0);

  bool distance_large_enough = start_to_goal.norm() > min_distance_to_step;
  bool goal_inside = IsGoalInsideStance();
  bool x_capture_inside = IsCapturePointInsideStance();

//  if (!x_capture_inside) {
//    throw std::runtime_error("Capture not inside");
//  }

  bool step_necessary =
//                        distance_large_enough ||
//                        !x_capture_inside ||
                        !goal_inside
//                      ||  (curr_state_.v.norm() > 0.1)
                     ;

  return step_necessary;
}

bool
StepSequencePlanner::StartWithStancePhase (const LegIDVec& step_sequence) const
{
  bool start_with_stance_phase = false;

  if (step_sequence.empty())
    start_with_stance_phase = true;
  else {
    bool zmp_inside = IsZmpInsideFirstStep(step_sequence.front());

    // so 4ls-phase not always inserted b/c of short time zmp constraints are ignored
    // when switching between disjoint support triangles.
//    if ( !zmp_inside  &&  curr_state_.v.norm() < 0.01)
    if (true)
      start_with_stance_phase = true;
  }

  return start_with_stance_phase;
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
StepSequencePlanner::IsZmpInsideFirstStep (LegID first_step) const
{
  // remove first swingleg from current stance
  VecFoothold first_stance = start_stance_;
  int idx_swingleg = Foothold::GetLastIndex(first_step, first_stance);
  first_stance.erase(first_stance.begin() + idx_swingleg);

  // fixme zero margins, since i actually allow violation of zmp constraint
  // don't want 4ls to be inserted there
  MarginValues margins = hyq::SupportPolygon::GetDefaultMargins();
//  margins.at(DIAG)/=2.;
  hyq::SupportPolygon supp(first_stance, margins);

  Eigen::Vector2d zmp = xpp::zmp::ZeroMomentPoint::CalcZmp(curr_state_.Make3D(), robot_height_);
  return supp.IsPointInside(zmp);
}

bool
StepSequencePlanner::IsCapturePointInsideStance () const
{
  // Jerry Pratt et al. : "Capture point: A step toward humanoid push recovery"
  static const double g = 9.80665; // gravity acceleration [m\s^2]
  // Defined in frame C located at contact point of inverted pendulum
  Vector2d x_capture_C = curr_state_.v.segment<2>(0)*std::sqrt(robot_height_/g);
  Vector2d x_capture_I = x_capture_C + curr_state_.p.segment<2>(0);

  MarginValues margins = hyq::SupportPolygon::GetDefaultMargins();
  hyq::SupportPolygon supp(start_stance_, margins);
  return supp.IsPointInside(x_capture_I);
}

bool
StepSequencePlanner::IsGoalInsideStance () const
{
  MarginValues margins = hyq::SupportPolygon::GetDefaultMargins();
  hyq::SupportPolygon supp(start_stance_, margins);

  return supp.IsPointInside(goal_state_.p);
}

} /* namespace hyq */
} /* namespace xpp */

