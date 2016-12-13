/**
 @file    motion_optimizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   The facade for the motion optimization, ROS independent
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_

#include <xpp/hyq/step_sequence_planner.h>
#include <xpp/opt/nlp_facade.h>
#include <xpp/opt/wb_traj_generator.h>
#include <xpp/hyq/hyq_state.h>

namespace xpp {
namespace opt {

/** Simplified interface to the complete motion optimization framework
  *
  * This is ROS independent.
  */
class MotionOptimizerFacade {
public:
  using State               = xpp::utils::StateLin3d;
  using StepSequencePlanner = xpp::hyq::StepSequencePlanner;
  using HyqStateVec         = std::vector<xpp::hyq::HyqState>;
  using HyqState            = xpp::hyq::HyqState;
  using VisualizerPtr       = std::shared_ptr<IVisualizer>;

  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  void Init(double max_step_length,
            double dt_nodes,
            double t_swing,
            double t_first_phase,
            double des_walking_height,
            double lift_height,
            double outward_swing,
            double trajectory_dt,
            VisualizerPtr visualizer);

  void OptimizeMotion();
  HyqStateVec GetTrajectory() const;

  void SetCurrent(const HyqState& curr);

  State goal_cog_;
  double t_left_; // time to reach goal

private:
  HyqState curr_state_;
  WBTrajGenerator<4> whole_body_mapper_; // zmp_ use hyq_robot_class for this
  NlpFacade nlp_facade_;
  StepSequencePlanner step_sequence_planner_;

  double max_step_length_;
  double dt_nodes_;
  double t_swing_;
  double t_first_phase_;
  double des_walking_height_;

  HyqStateVec optimized_trajectory_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
