/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the ROS interface to motion optimizer.
 */

#ifndef XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_
#define XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_

#include <xpp/opt/motion_optimizer_facade.h>

#include <xpp_msgs/CurrentInfo.h>  // receive from robot
#include <xpp_msgs/UserCommand.h>  // receive from user
#include "ros_visualizer.h"

namespace xpp {
namespace ros {

class NlpOptimizerNode {
public:
  using OptVisualizerPtr = std::shared_ptr<RosVisualizer>;
  using CurrentInfoMsg   = xpp_msgs::CurrentInfo;
  using UserCommandMsg   = xpp_msgs::UserCommand;
  using MotionOptimizer  = xpp::opt::MotionOptimizerFacade;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:
  void PublishTrajectory();
  void CurrentStateCallback(const CurrentInfoMsg& msg);
  void UserCommandCallback(const UserCommandMsg& msg);

  ::ros::Subscriber user_command_sub_;
  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher trajectory_pub_;

  OptVisualizerPtr ros_marker_visualizer_;
  MotionOptimizer motion_optimizer_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
