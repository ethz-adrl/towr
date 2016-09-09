/**
 @file    nlp_user_input_node.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Declares the NlpUserInputNode class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_

#include <xpp/utils/geometric_structs.h>

#include <xpp_opt/GetStateLin3d.h>
#include <xpp_opt/StateLin3d.h>
#include <keyboard/Key.h>
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
  using State = xpp::utils::Point3d;
  using StateMsg = xpp_opt::StateLin3d;
  using KeyboardMsg = keyboard::Key;
  using GoalSrv = xpp_opt::GetStateLin3d;

  NlpUserInputNode ();
  virtual ~NlpUserInputNode ();

private:
  void CallbackKeyboard(const keyboard::Key& msg);
  bool GetGoalService(GoalSrv::Request& req, GoalSrv::Response& res);

  State goal_cog_;

  ::ros::Subscriber goal_key_sub_;
  ::ros::Publisher  goal_state_pub_;
  ::ros::Publisher  rviz_publisher_;
  ::ros::ServiceServer get_goal_srv_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_ */
