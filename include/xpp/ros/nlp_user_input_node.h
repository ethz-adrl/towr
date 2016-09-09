/**
 @file    nlp_user_input_node.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_

#include <xpp/utils/geometric_structs.h>

#include <xpp_opt/StateLin3d.h>
#include <keyboard/Key.h>
#include <ros/ros.h>

namespace xpp {
namespace ros {

class NlpUserInputNode {
public:
  using State = xpp::utils::Point3d;
  using StateMsg = xpp_opt::StateLin3d;
  using KeyboardMsg = keyboard::Key;

  NlpUserInputNode ();
  virtual ~NlpUserInputNode ();

private:
  void CallbackKeyboard(const keyboard::Key& msg);

  State goal_cog_;

  ::ros::Subscriber goal_key_sub_;
  ::ros::Publisher  goal_state_pub_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_USER_INPUT_NODE_H_ */
