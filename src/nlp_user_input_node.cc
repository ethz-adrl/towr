/**
 @file    nlp_user_input_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Brief description
 */

#include <xpp/ros/nlp_user_input_node.h>
#include <xpp/ros/ros_helpers.h>

namespace xpp {
namespace ros {

NlpUserInputNode::NlpUserInputNode ()
{
  ::ros::NodeHandle n;
  goal_key_sub_ = n.subscribe("/keyboard/keydown", 1,
                                &NlpUserInputNode::CallbackKeyboard, this);

  goal_state_pub_ = n.advertise<StateMsg>("goal_state", 1);

  // Get Starting goal state from server
  goal_cog_.p.x() = RosHelpers::GetDoubleFromServer("/xpp/goal_state_x");
  goal_cog_.p.y() = RosHelpers::GetDoubleFromServer("/xpp/goal_state_y");

  // publish values once initially
  goal_state_pub_.publish(RosHelpers::XppToRos(goal_cog_));
}

NlpUserInputNode::~NlpUserInputNode ()
{
  // TODO Auto-generated destructor stub
}

void
NlpUserInputNode::CallbackKeyboard (const keyboard::Key& msg)
{
  const static double dx = 0.1;
  const static double dy = 0.1;

  switch (msg.code) {
    case msg.KEY_UP:
      goal_cog_.p.x() += dx;
      break;
    case msg.KEY_DOWN:
      goal_cog_.p.x() -= dx;
      break;
    case msg.KEY_RIGHT:
      goal_cog_.p.y() -= dy;
      break;
    case msg.KEY_LEFT:
      goal_cog_.p.y() += dy;
      break;
    default:
      break;
  }

  goal_state_pub_.publish(RosHelpers::XppToRos(goal_cog_));
  ROS_INFO_STREAM("Publishing goal state : " << goal_cog_);
}

} /* namespace ros */
} /* namespace xpp */
