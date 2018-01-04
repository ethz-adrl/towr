/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the ROS interface to motion optimizer.
 */

#ifndef XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_
#define XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_

#include <string>

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/RobotParameters.h>

#include <xpp_opt_ros/UserCommand.h>    // receive from user
#include <xpp_opt/motion_optimizer_facade.h>

namespace xpp {

class NlpOptimizerNode {
public:
  using UserCommand = xpp_opt_ros::UserCommand;

  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () = default;

private:
  void OptimizeMotion();
  void CurrentStateCallback(const xpp_msgs::RobotStateCartesian&);

  void UserCommandCallback(const UserCommand&);

  ::ros::Subscriber user_command_sub_;
  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher cart_trajectory_pub_;
  ::ros::Publisher opt_parameters_pub_;

  MotionOptimizerFacade motion_optimizer_;
  double dt_; ///< discretization of output trajectory (1/TaskServoHz)

  UserCommand user_command_msg_;

  bool first_callback_ = true;

  std::string rosbag_folder_; ///< folder to save bags
  std::string bag_name_ = "/optimal_traj.bag";
  std::vector<RobotStateCartesian> curr_states_log_;
  bool save_current_state_ = false;

  xpp_msgs::RobotStateCartesianTrajectory BuildTrajectoryMsg() const;

  void SaveOptimizationAsRosbag(const std::string& bag_name, bool include_iterations=false) const;
  void SaveTrajectoryInRosbag (rosbag::Bag&, const std::vector<RobotStateCartesian>& traj,
                               const std::string& topic) const;
  xpp_msgs::RobotParameters BuildOptParametersMsg() const;
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
