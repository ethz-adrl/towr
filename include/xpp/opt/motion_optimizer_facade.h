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
#include <xpp/opt/robot_state_cartesian.h>
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
  using RobotState    = xpp::opt::RobotStateCartesian; //xpp::hyq::HyqState;//
  using RobotStateVec = std::vector<RobotState>;
  using MotionTypePtr = std::shared_ptr<MotionParameters>;
  using PhaseVec      = std::vector<MotionPhase>;

  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  void SetVisualizer(VisualizerPtr visualizer);

  void OptimizeMotion(NlpSolver solver);
  RobotStateVec GetTrajectory(double dt);

  void BuildOptimizationStartState(const RobotState& curr);

  State goal_cog_;
  double t_left_; // time to reach goal

  void SetMotionType(MotionTypeID);


private:
  RobotState opt_start_state_;
  RobotState state_second_phase_; ///< the current goal to reach

  NlpFacade nlp_facade_;
  StepSequencePlanner step_sequence_planner_;
  MotionTypePtr motion_type_;
  PhaseVec motion_phases_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
