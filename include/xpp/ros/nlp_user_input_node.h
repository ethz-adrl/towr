/**
 @file    nlp_user_input_node.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Declares the NlpUserInputNode class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Vector3.h>
#include <keyboard/Key.h>
#include <sensor_msgs/Joy.h>

#include <xpp/state.h>


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

  NlpUserInputNode ();
  virtual ~NlpUserInputNode () {};
  void PublishCommand();

private:
  ::ros::Subscriber key_sub_;
  void CallbackKeyboard(const KeyboardMsg& msg);
  ::ros::Publisher  user_command_pub_;


  State3dEuler goal_geom_;
  int terrain_id_;
  bool replay_trajectory_ = false;
  bool use_solver_snopt_ = false;
  bool optimize_ = false;
  int gait_type_;
  double total_duration_ = 3.0;


//  JoyMsg joy_msg_;
//  ::ros::Subscriber joy_sub_;
//  void CallbackJoy(const JoyMsg& msg);
//  void ModifyGoalJoy();
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_ */
