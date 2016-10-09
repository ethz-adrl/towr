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
NlpUserInputNode::CallbackKeyboard (const KeyboardMsg& msg)
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
    default:
      break;
  }

  ROS_INFO_STREAM("Set goal state to " << goal_cog_.Get2D().p.transpose() << "?");

  static std::map<uint, NlpUserInputNode::Command> keyboard_command_map_ =
  {
    {NlpUserInputNode::KeyboardMsg::KEY_RETURN, NlpUserInputNode::Command::kSetGoal,    },
    {NlpUserInputNode::KeyboardMsg::KEY_w     , NlpUserInputNode::Command::kStartWalking}
  };

  bool key_in_map = keyboard_command_map_.find(msg.code) != keyboard_command_map_.end();
  if (key_in_map)
    command_ = keyboard_command_map_.at(msg.code);

}

void
NlpUserInputNode::CallbackJoy (const JoyMsg& msg)
{
  joy_msg_ = msg;

  enum JoyButtons {X=0, A, B, Y};
  static std::map<JoyButtons, NlpUserInputNode::Command> joy_command_map_
  {
    {JoyButtons::X, NlpUserInputNode::Command::kSetGoal},
    {JoyButtons::A, NlpUserInputNode::Command::kSetGoal},
    {JoyButtons::B, NlpUserInputNode::Command::kSetGoal},
    {JoyButtons::Y, NlpUserInputNode::Command::kStartWalking},
  };

  // the last pushed button
  for (auto i : {X,A,B,Y})
    if (joy_msg_.buttons[i] == 1)
      command_ = joy_command_map_.at(i);

  ROS_INFO_STREAM("Current goal set to " << goal_cog_.Get2D().p.transpose() << ".");
}

void
NlpUserInputNode::ModifyGoalJoy ()
{
  enum Axis {L_LEFT = 0, L_FORWARD, R_LEFT, R_FORWARD};

  double max_vel = 0.5; // [m/s]

  double vel_x = max_vel*joy_msg_.axes[L_FORWARD];
  double vel_y = max_vel*joy_msg_.axes[L_LEFT];

  // integrate velocity
  goal_cog_.p.x() += vel_x * 1.0/kLoopRate_;
  goal_cog_.p.y() += vel_y * 1.0/kLoopRate_;
}

void NlpUserInputNode::PublishCommand()
{
  if (!joy_msg_.axes.empty())
    ModifyGoalJoy();


  // send out goal state only if changed w.r.t previous
  if (goal_cog_.p != goal_prev_.p) {
    goal_state_pub_.publish(RosHelpers::XppToRos(goal_cog_));
  }
  goal_prev_ = goal_cog_;

  switch (command_) {
    case Command::kSetGoal:
      ROS_INFO_STREAM("Sending out desired goal state");
      break;
    case Command::kStartWalking:
      ROS_INFO_STREAM("Sending out walking command");
      walk_command_pub_.publish(std_msgs::Empty());
      break;
    default: // no command
      break;
  }

  command_ = Command::kNoCommand;
}

void
NlpUserInputNode::PublishRviz () const
{
  visualization_msgs::MarkerArray msg_rviz;
  MarkerArrayBuilder msg_builder_;
  msg_builder_.AddPoint(msg_rviz, goal_cog_.Get2D().p, "goal",visualization_msgs::Marker::CUBE);
  rviz_publisher_.publish(msg_rviz);
}

bool
NlpUserInputNode::GetGoalService (GoalSrv::Request& req, GoalSrv::Response& res)
{
  res.state = RosHelpers::XppToRos(goal_cog_);
  return true;
}

} /* namespace ros */
} /* namespace xpp */
