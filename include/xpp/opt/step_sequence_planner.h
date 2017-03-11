/**
 @file    step_sequence_planner.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the interface to the StepSequencePlanner.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_

#include <xpp/state.h>
#include <xpp/contact.h>
#include <xpp/opt/motion_parameters.h>
#include <memory>

namespace xpp {
namespace opt {

/** Plans the sequence of steps (LH, LF, ...) for a given optimization problem.
  *
  * This is information that has to be passed to the NLP optimizer beforehand.
  * This class is specific for HyQ and must be adapted for different robots.
  * It contains all the information needed to determine the step sequence.
  */
class StepSequencePlanner {
public:
  using SwingLegsInPhase  = MotionParameters::SwinglegPhase;
  using AllPhaseSwingLegs = std::vector<SwingLegsInPhase>;
  using StartStance       = std::vector<Contact>;
  using State             = StateLin2d;
  using MotionParamsPtr   = std::shared_ptr<MotionParameters>;

  StepSequencePlanner ();
  virtual ~StepSequencePlanner ();

  /** Necessary information to determine the step sequence.
    *
    * @param the current pos/vel/acc of the robot.
    * @param the desired pos/vel/acc that the robot should achieve.
    */
  void Init(const State& curr, const State& goal);

  /** Defines the endeffectors in swing for each motion phase
    */
  AllPhaseSwingLegs DetermineStepSequence(const MotionParamsPtr&);

private:
  State curr_state_;
  State goal_state_;
  StartStance start_stance_;
  double robot_height_;
  double max_step_length_;
  int curr_swingleg_; // this could also be no swingleg (stance phase)
};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_ */
