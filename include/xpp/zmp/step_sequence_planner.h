/**
 @file    step_sequence_planner.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_

#include <xpp/hyq/leg_data_map.h>
#include <xpp/hyq/foothold.h>
#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace hyq {

/** Plans the sequence of steps (LH, LF, ...) for a given optimization problem
  *
  * This is information that has to be passed to the NLP optimizer beforehand.
  * This class is specific for HyQ and must be adapted for different robots.
  * It contains all the information needed to determine the step sequence.
  */
class StepSequencePlanner {
public:
  typedef std::vector<LegID> LegIDVec;
  typedef std::vector<Foothold> VecFoothold;
  typedef xpp::utils::Point3d State;

  StepSequencePlanner (const State& curr, const State& goal, int swingleg);
  virtual ~StepSequencePlanner ();

  bool GoalTooCloseToStep() const;
  bool StartWithStancePhase(VecFoothold curr_stance_,double robot_height,LegID first_swingleg) const;
  LegIDVec DetermineStepSequence();

private:
  LegID NextSwingLeg(LegID curr) const;

  State curr_state_;
  State goal_state_;
  int curr_swing_leg_;   // this can also be a four-leg-support indicator
  LegID prev_swing_leg_; // this is always a swingleg
  const double min_distance_to_step_ = 0.08;//m
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_ */
