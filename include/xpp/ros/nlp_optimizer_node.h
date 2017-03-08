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

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

namespace xpp {
namespace ros {

class NlpOptimizerNode {
public:
  using CurrentInfoMsg   = xpp_msgs::CurrentInfo;
  using UserCommandMsg   = xpp_msgs::UserCommand;
  using MotionOptimizer  = xpp::opt::MotionOptimizerFacade;
  using NlpSolver        = xpp::opt::NlpSolver;

  using PoseMsg        = geometry_msgs::PoseStamped;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:

  /** sends this info the the walking controller **/
  void PublishTrajectory();
  void OptimizeMotion();
  void CurrentStateCallback(const CurrentInfoMsg& msg);
  void UserCommandCallback(const UserCommandMsg& msg);

  ::ros::Subscriber user_command_sub_;
  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher trajectory_pub_;
  ::ros::Publisher contacts_pub_;

  MotionOptimizer motion_optimizer_;
  double dt_; ///< discretization of output trajectory (1/TaskServoHz)
  NlpSolver solver_type_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
