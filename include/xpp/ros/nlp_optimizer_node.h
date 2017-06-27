/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the ROS interface to motion optimizer.
 */

#ifndef XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_
#define XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_

#include <xpp/opt/motion_optimizer_facade.h>

#include <xpp_msgs/RobotStateCartesian.h>  // receive from robot
#include <xpp_msgs/UserCommand.h>          // receive from user
#include <ros/ros.h>

namespace xpp {
namespace ros {

class NlpOptimizerNode {
public:
  using StateMsg              = xpp_msgs::RobotStateCartesian;
  using UserCommandMsg        = xpp_msgs::UserCommand;
  using MotionOptimizerFacade = xpp::opt::MotionOptimizerFacade;
  using NlpSolver             = xpp::opt::NlpSolver;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:
  void PublishTrajectory();
  void OptimizeMotion();
  void CurrentStateCallback(const StateMsg& msg);
  void UserCommandCallback(const UserCommandMsg& msg);

  ::ros::Subscriber user_command_sub_;
  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher cart_trajectory_pub_;

  MotionOptimizerFacade motion_optimizer_;
  double dt_; ///< discretization of output trajectory (1/TaskServoHz)
  NlpSolver solver_type_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
