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

static std::map<uint, NlpUserInputNode::Command> keyboard_map_ =
{
  {NlpUserInputNode::KeyboardMsg::KEY_RETURN, NlpUserInputNode::Command::kSetGoal,    },
  {NlpUserInputNode::KeyboardMsg::KEY_w     , NlpUserInputNode::Command::kStartWalking}
};

enum JoyButtons {X=0, A, B, Y, NO_BUTTON};
static std::vector<JoyButtons> AllJoyButtons = {JoyButtons::X,JoyButtons::A,
                                                JoyButtons::B,JoyButtons::Y};
static std::map<JoyButtons, NlpUserInputNode::Command> joy_map_
{
  {JoyButtons::X, NlpUserInputNode::Command::kSetGoal},
  {JoyButtons::A, NlpUserInputNode::Command::kSetGoal},
  {JoyButtons::B, NlpUserInputNode::Command::kSetGoal},
  {JoyButtons::Y, NlpUserInputNode::Command::kStartWalking},
};


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

  bool key_in_map = keyboard_map_.find(msg.code) != keyboard_map_.end();
  if (key_in_map)
    command_ = keyboard_map_.at(msg.code);

}

void
NlpUserInputNode::CallbackJoy (const JoyMsg& msg)
{
  prev_msg_ = msg;

  // the last pushed button
  JoyButtons pushed_button = JoyButtons::NO_BUTTON;
  for (auto i : AllJoyButtons) {
    if (prev_msg_.buttons[i] == 1)
      pushed_button = i;
  }

  if (pushed_button != JoyButtons::NO_BUTTON) {
    command_ = joy_map_.at(pushed_button);
  }

  ROS_INFO_STREAM("Current goal set to " << goal_cog_.Get2D().p.transpose() << ".");
}

void
NlpUserInputNode::ModifyGoalJoy ()
{
  enum Axis {L_LEFT = 0, L_FORWARD, R_LEFT, R_FORWARD};

  double vel_x = 0.5*prev_msg_.axes[L_FORWARD];
  double vel_y = 0.5*prev_msg_.axes[L_LEFT];

  // integrate velocity
  goal_cog_.p.x() += vel_x * 1.0/kLoopRate_;
  goal_cog_.p.y() += vel_y * 1.0/kLoopRate_;
}

void NlpUserInputNode::PublishCommand()
{

  if (!prev_msg_.axes.empty()) {
    ModifyGoalJoy();
  }

  switch (command_) {
    case Command::kSetGoal:
      goal_state_pub_.publish(RosHelpers::XppToRos(goal_cog_));
      ROS_INFO_STREAM("Sending out desired goal state: " << goal_cog_.Get2D().p.transpose() << ".");
      break;
    case Command::kStartWalking:
      walk_command_pub_.publish(std_msgs::Empty());
      ROS_INFO_STREAM("Sending out walking command");
      break;
    case Command::kNoCommand: // do nothing
    default: break;
  }

  command_ = Command::kNoCommand;

  // send out goal state to rviz
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
