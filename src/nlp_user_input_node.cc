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

#include <std_msgs/Empty.h>         // send to trigger walking
#include <xpp_msgs/UserCommand.h>   // send to optimizer node

namespace xpp {
namespace ros {

using UserCommandMsg = xpp_msgs::UserCommand;

NlpUserInputNode::NlpUserInputNode ()
    :t_max_left_(1.0) //s
{
  ::ros::NodeHandle n;
  key_sub_ = n.subscribe("/keyboard/keydown", 1, &NlpUserInputNode::CallbackKeyboard, this);
  joy_sub_ = n.subscribe("/joy", 1, &NlpUserInputNode::CallbackJoy, this);

  user_command_pub_ = n.advertise<UserCommandMsg>(xpp_msgs::goal_state_topic, 1);

  // publish goal zero initially
  goal_cog_.p.setZero();
  motion_type_ = opt::TrottID;
  replay_trajectory_ = false;
  UserCommandMsg msg;
  msg.t_left = t_max_left_;
  msg.goal = RosHelpers::XppToRos(goal_cog_);
  user_command_pub_.publish(msg);

  // start walking command
  walk_command_pub_ = n.advertise<std_msgs::Empty>(xpp_msgs::start_walking_topic,1);
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
    case msg.KEY_RETURN:
      command_ = Command::kSetGoal;
      break;
    case msg.KEY_g:
      ROS_INFO_STREAM("Start Walking command sent to controller");
      command_ = Command::kStartWalking;
      break;
    case msg.KEY_w:
      ROS_INFO_STREAM("Motion type set to Walking");
      motion_type_ = opt::WalkID;
      motion_type_change_ = true;
      break;
    case msg.KEY_t:
      ROS_INFO_STREAM("Motion type set to Trotting");
      motion_type_ = opt::TrottID;
      motion_type_change_ = true;
      break;
    case msg.KEY_b:
      ROS_INFO_STREAM("Motion type set to Bounding");
      motion_type_ = opt::BoundID;
      motion_type_change_ = true;
      break;
    case msg.KEY_c:
      ROS_INFO_STREAM("Motion type set to Camel");
      motion_type_ = opt::CamelID;
      motion_type_change_ = true;
      break;
    case msg.KEY_p:
      ROS_INFO_STREAM("Motion type set to Push Recovery");
      motion_type_ = opt::PushRecID;
      motion_type_change_ = true;
      break;
    case msg.KEY_r:
      ROS_INFO_STREAM("Replaying already optimized trajectory");
      replay_trajectory_ = true;
      break;
    default:
      break;
  }
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
    t_left_ = t_max_left_;

  UserCommandMsg msg;
  msg.t_left             = t_left_;
  msg.goal               = RosHelpers::XppToRos(goal_cog_);
  msg.motion_type        = motion_type_;
  msg.motion_type_change = motion_type_change_;
  msg.replay_trajectory  = replay_trajectory_;
  user_command_pub_.publish(msg);

  switch (command_) {
    case Command::kSetGoal: {
      ROS_INFO_STREAM("Sending out desired goal state");
      break;
    }
    case Command::kStartWalking: {
      ROS_INFO_STREAM("Sending out walking command");
      walk_command_pub_.publish(std_msgs::Empty());
      break;
    }
    default: // no command
      break;
  }

  goal_cog_prev_ = goal_cog_;
  command_ = Command::kNoCommand;
  replay_trajectory_  = false;
  motion_type_change_ = false;
}

} /* namespace ros */
} /* namespace xpp */
