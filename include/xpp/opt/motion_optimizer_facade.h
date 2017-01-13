/**
 @file    motion_optimizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   The facade for the motion optimization, ROS independent
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_

#include <xpp/opt/step_sequence_planner.h>
#include <xpp/opt/nlp_facade.h>
#include <xpp/hyq/hyq_state.h>
#include "motion_parameters.h"

namespace xpp {
namespace opt {

/** Simplified interface to the complete motion optimization framework
  *
  * This is ROS independent.
  */
class MotionOptimizerFacade {
public:
  using State         = xpp::utils::StateLin3d;
  using VisualizerPtr = std::shared_ptr<IVisualizer>;
  using HyqState      = xpp::hyq::HyqState;
  using HyqStateVec   = HyqState::StateJVec;
  using MotionTypePtr = std::shared_ptr<MotionParameters>;
  using PhaseVec      = std::vector<MotionPhase>;

  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  void SetVisualizer(VisualizerPtr visualizer);

  void OptimizeMotion(NlpSolver solver);
  HyqStateVec GetTrajectory(double dt) const;

  void SetCurrent(const HyqState& curr);

  State goal_cog_;
  double t_left_; // time to reach goal

  void SetMotionType(MotionTypeID);


private:
  HyqState curr_state_;
  NlpFacade nlp_facade_;
  StepSequencePlanner step_sequence_planner_;
  MotionTypePtr motion_type_;

  PhaseVec motion_phases_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
