/**
 @file    nlp_user_input_node.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Declares the NlpUserInputNode class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_

#include <xpp/utils/base_state.h>

#include <xpp_opt/GetStateLin3d.h>
#include <xpp_msgs/StateLin3d.h>
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
  using State = xpp::utils::BaseLin3d;
  using StateMsg = xpp_msgs::StateLin3d;
  using KeyboardMsg = keyboard::Key;
  using JoyMsg = sensor_msgs::Joy;
  using GoalSrv = xpp_opt::GetStateLin3d;

  NlpUserInputNode ();
  virtual ~NlpUserInputNode ();

private:
  void CallbackKeyboard(const KeyboardMsg& msg);
  void CallbackJoy(const JoyMsg& msg);
  bool GetGoalService(GoalSrv::Request& req, GoalSrv::Response& res);

  State goal_cog_;

  ::ros::Subscriber key_sub_;
  ::ros::Subscriber joy_sub_;

  ::ros::Publisher  goal_state_pub_;
  ::ros::Publisher  walk_command_pub_; // tells the robot to start walking
  ::ros::Publisher  rviz_publisher_;
  ::ros::ServiceServer get_goal_srv_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_ */
