/**
 @file    nlp_user_input_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Defines the NlpUserInputNode class
 */

#include <xpp/ros/nlp_user_input_node.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/marker_array_builder.h>
#include <xpp/ros/topic_names.h>
#include <xpp/ros/ros_helpers.h>

//#include <std_msgs/Empty.h>         // send to trigger walking
#include <xpp_msgs/UserCommand.h>   // send to optimizer node

namespace xpp {
namespace ros {

using UserCommandMsg = xpp_msgs::UserCommand;

NlpUserInputNode::NlpUserInputNode ()
{
  ::ros::NodeHandle n;
  key_sub_ = n.subscribe("/keyboard/keydown", 1, &NlpUserInputNode::CallbackKeyboard, this);
  joy_sub_ = n.subscribe("/joy", 1, &NlpUserInputNode::CallbackJoy, this);

  user_command_pub_ = n.advertise<UserCommandMsg>(xpp_msgs::goal_state_topic, 1);

  // publish goal zero initially
  t_left_ = RosHelpers::GetDoubleFromServer("xpp/stance_time_initial");
  goal_cog_.p.setZero();
  UserCommandMsg msg;
  msg.t_left = t_left_;
  msg.goal = RosHelpers::XppToRos(goal_cog_);
  user_command_pub_.publish(msg);

  // start walking command
//  walk_command_pub_ = n.advertise<std_msgs::Empty>(xpp_msgs::start_walking_topic,1);
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

  if (goal_cog_ != goal_cog_prev_)
    t_left_ = RosHelpers::GetDoubleFromServer("xpp/stance_time_initial");

  UserCommandMsg msg;
  msg.t_left = t_left_;
  msg.goal = RosHelpers::XppToRos(goal_cog_);
  user_command_pub_.publish(msg);

//  switch (command_) {
//    case Command::kSetGoal: {
//      ROS_INFO_STREAM("Sending out desired goal state");
//      break;
//    }
//    case Command::kStartWalking: {
//      ROS_INFO_STREAM("Sending out walking command");
//      walk_command_pub_.publish(std_msgs::Empty());
//      break;
//    }
//    default: // no command
//      break;
//  }

  goal_cog_prev_ = goal_cog_;
  command_ = Command::kNoCommand;
}

} /* namespace ros */
} /* namespace xpp */
