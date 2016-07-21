/**
 @file    step_sequence_planner.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the interface to the StepSequencePlanner.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_

#include <xpp/hyq/leg_data_map.h>
#include <xpp/hyq/foothold.h>
#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace hyq {

/** Plans the sequence of steps (LH, LF, ...) for a given optimization problem.
  *
  * This is information that has to be passed to the NLP optimizer beforehand.
  * This class is specific for HyQ and must be adapted for different robots.
  * It contains all the information needed to determine the step sequence.
  */
class StepSequencePlanner {
public:
  typedef std::vector<LegID> LegIDVec;
  typedef std::vector<Foothold> VecFoothold;
  typedef xpp::utils::Point2d State;

  StepSequencePlanner ();
  virtual ~StepSequencePlanner ();

  /** @brief Set the current and the goal state */
  void SetCurrAndGoal(const State& curr, const State& goal);

  /** Determines whether an initial stance phase is inserted.
    *
    * This is necessary, if the initial ZMP is outside the area of support
    * of the first step, so the optimizer would not be able to find a solution.
    *
    * @param start_stance the 4 footholds right before swining the first leg.
    * @param robot_height the walking height [m] of the robot.
    * @param first_swingleg the first leg to be swung.
    * @return true if stance phase must be inserted.
    */
  bool StartWithStancePhase(const VecFoothold& start_stance,
                            double robot_height,
                            LegID first_swingleg) const;

  /** Determines the sequence of steps (LH, LF, ...) to take.
    *
    * @param curr_swing_leg The current swing leg, optimization then starts with next one.
    * @return vector of swinglegs.
    */
  LegIDVec DetermineStepSequence(int curr_swing_leg);

private:
  bool GoalTooCloseToStep() const;
  LegID NextSwingLeg(LegID curr) const;

  State curr_state_;
  State goal_state_;
  LegID prev_swing_leg_; // this is always a swingleg

};

} /* namespace hyq */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_ */
