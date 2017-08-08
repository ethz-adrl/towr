/**
 @file    nlp_user_input_node.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Declares the NlpUserInputNode class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_

#include <xpp/opt/motion_parameters.h>
#include <xpp/state.h>

#include <geometry_msgs/Vector3.h>
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
  using KeyboardMsg = keyboard::Key;
  using JoyMsg      = sensor_msgs::Joy;
  using InitVel     = geometry_msgs::Vector3;

  enum class Command { kSetGoal, kStartWalking, kNoCommand } command_ = Command::kNoCommand;

  NlpUserInputNode ();
  virtual ~NlpUserInputNode ();
  void PublishCommand();

private:
  void CallbackKeyboard(const KeyboardMsg& msg);
  void CallbackJoy(const JoyMsg& msg);

  void ModifyGoalJoy();

  State3dEuler goal_geom_;

  bool replay_trajectory_;
  bool use_solver_snopt_;
  InitVel velocity_disturbance_;

  ::ros::Subscriber key_sub_;
  ::ros::Subscriber joy_sub_;
  JoyMsg joy_msg_;

  ::ros::Publisher  user_command_pub_;
  ::ros::Publisher  walk_command_pub_; // tells the robot to start walking
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_ */
