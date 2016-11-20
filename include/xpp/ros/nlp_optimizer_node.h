/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the interface to the ROS node that initializes/calls the NLP optimizer.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_

#include <xpp/opt/motion_optimizer_facade.h>

#include <xpp_msgs/HyqState.h>         // receive
#include <xpp_msgs/StateLin3d.h>       // receive
#include "ros_visualizer.h"

namespace xpp {
namespace ros {

class NlpOptimizerNode {
public:
  using OptVisualizerPtr = std::shared_ptr<RosVisualizer>;
  using HyqStateMsg      = xpp_msgs::HyqState;
  using StateMsg         = xpp_msgs::StateLin3d;

  using MotionOptimizer  = xpp::opt::MotionOptimizerFacade;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:
  void PublishTrajectory() const;
  void CurrentStateCallback(const HyqStateMsg& msg);
  void GoalStateCallback(const StateMsg& msg);

  ::ros::Subscriber goal_state_sub_;
  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher trajectory_pub_;

  OptVisualizerPtr ros_visualizer_;
  MotionOptimizer motion_optimizer_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
