/**
 @file    nlp_user_input_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Defines the NlpUserInputNode class
 */

#include <xpp/ros/nlp_user_input_node.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/marker_array_builder.h>
#include <std_msgs/Empty.h>
#include <xpp_msgs/topic_names.h>

namespace xpp {
namespace ros {

NlpUserInputNode::NlpUserInputNode ()
{
  ::ros::NodeHandle n;
  key_sub_ = n.subscribe("/keyboard/keydown", 1, &NlpUserInputNode::CallbackKeyboard, this);
  joy_sub_ = n.subscribe("/joy", 1, &NlpUserInputNode::CallbackJoy, this);

  goal_state_pub_ = n.advertise<StateMsg>(xpp_msgs::goal_state_topic, 1);

  // start walking command
  walk_command_pub_ = n.advertise<std_msgs::Empty>(xpp_msgs::start_walking_topic,1);

  rviz_publisher_ = n.advertise<visualization_msgs::MarkerArray>("optimization_fixed", 1);
  get_goal_srv_   = n.advertiseService("get_goal_state", &NlpUserInputNode::GetGoalService, this);

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
  const static double dy = 0.05;

  switch (msg.code) {
    case msg.KEY_LEFT:
      goal_cog_.p.x() += dx;
      break;
    case msg.KEY_RIGHT:
      goal_cog_.p.x() -= dx;
      break;
    case msg.KEY_UP:
      goal_cog_.p.y() -= dy;
      break;
    case msg.KEY_DOWN:
      goal_cog_.p.y() += dy;
      break;
    case msg.KEY_RETURN:
      goal_state_pub_.publish(RosHelpers::XppToRos(goal_cog_));
      ROS_INFO_STREAM("Goal state set to " << goal_cog_.Get2D().p.transpose() << ".");
      break;
    case msg.KEY_w:
      walk_command_pub_.publish(std_msgs::Empty());
      ROS_INFO_STREAM("Walking command sent");
      break;
    default:
      break;
  }

  ROS_INFO_STREAM("Set goal state to " << goal_cog_.Get2D().p.transpose() << "?");

  // send out goal state to rviz
  visualization_msgs::MarkerArray msg_rviz;
  MarkerArrayBuilder msg_builder_;
  msg_builder_.AddPoint(msg_rviz, goal_cog_.Get2D().p, "goal",
                        visualization_msgs::Marker::CUBE);
  rviz_publisher_.publish(msg_rviz);
}

void
NlpUserInputNode::CallbackJoy (const JoyMsg& msg)
{
  enum Buttons {X=0, A, B, Y};
  if (msg.buttons[A] == 1) {
    ROS_INFO_STREAM("A pressed");
  }
}

bool
NlpUserInputNode::GetGoalService (GoalSrv::Request& req, GoalSrv::Response& res)
{
  res.state = RosHelpers::XppToRos(goal_cog_);
  return true;
}

} /* namespace ros */
} /* namespace xpp */
