/**
 @file    nlp_user_input_node.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Declares the NlpUserInputNode class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_

#include <xpp_msgs/GetStateLin3d.h>
#include <xpp/utils/state.h>
#include <xpp/opt/motion_type.h>

#include <keyboard/Key.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace xpp {
namespace ros {

/** @brief Provides user specified information about the NLP to the optimizer
  *
  * This includes high level input about where to go (e.g. converting
  * joystick information to a goal state)
  */
class NlpUserInputNode {
public:
  using State       = xpp::utils::StateLin3d;
  using KeyboardMsg = keyboard::Key;
  using JoyMsg      = sensor_msgs::Joy;
  using GoalSrv     = xpp_msgs::GetStateLin3d;
  using MotionType  = xpp::opt::MotionTypeID;

  enum class Command { kSetGoal, kStartWalking, kNoCommand } command_ = Command::kNoCommand;

  NlpUserInputNode ();
  virtual ~NlpUserInputNode ();
  void PublishCommand();

  const int kLoopRate_ = 30; ///< frequency for sending out control commands
  double t_left_;
  const double t_max_left_;

private:
  void CallbackKeyboard(const KeyboardMsg& msg);
  void CallbackJoy(const JoyMsg& msg);

  void ModifyGoalJoy();

  State goal_cog_;
  State goal_cog_prev_;

  MotionType motion_type_;

  ::ros::Subscriber key_sub_;
  ::ros::Subscriber joy_sub_;
  JoyMsg joy_msg_;

  ::ros::Publisher  user_command_pub_;
  ::ros::Publisher  walk_command_pub_; // tells the robot to start walking
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_ */
