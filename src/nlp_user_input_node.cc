/**
 @file    nlp_user_input_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 9, 2016
 @brief   Defines the NlpUserInputNode class
 */

#include <xpp/ros/nlp_user_input_node.h>

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <ros/node_handle.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Empty.h>         // send to trigger walking
#include <xpp_msgs/UserCommand.h>   // send to optimizer node

#include <xpp/ros/ros_conversions.h>
#include <xpp/ros/topic_names.h>
#include <xpp/height_map.h>
#include <xpp/quadruped_gait_generator.h>

namespace xpp {
namespace ros {


NlpUserInputNode::NlpUserInputNode ()
{
  ::ros::NodeHandle n;
  key_sub_ = n.subscribe("/keyboard/keydown", 1, &NlpUserInputNode::CallbackKeyboard, this);
//  joy_sub_ = n.subscribe("/joy", 1, &NlpUserInputNode::CallbackJoy, this);

  user_command_pub_ = n.advertise<xpp_msgs::UserCommand>(xpp_msgs::user_command, 1);

  // publish goal zero initially
  goal_geom_.lin.p_.setZero();
  goal_geom_.lin.p_ << 1.3, 0.0, 0.46;
  goal_geom_.ang.p_ << 0.0, 0.0, 0.0; // roll, pitch, yaw angle applied Z->Y'->X''


  terrain_id_ = opt::HeightMap::FlatID;
  gait_type_ = quad::QuadrupedGaitGenerator::Trot;

//  UserCommandMsg msg;
//  msg.goal_lin = RosConversions::XppToRos(goal_geom_.lin);
//  msg.goal_ang = RosConversions::XppToRos(goal_geom_.ang);
//  user_command_pub_.publish(msg);
//
//  // start walking command
//  walk_command_pub_ = n.advertise<std_msgs::Empty>(xpp_msgs::start_walking_topic,1);
}

void
NlpUserInputNode::CallbackKeyboard (const KeyboardMsg& msg)
{
  const static double d_lin = 0.1;  // [m]
  const static double d_ang = 0.25; // [rad]

  switch (msg.code) {
    // desired goal positions
    case msg.KEY_RIGHT:
      goal_geom_.lin.p_.x() -= d_lin;
      break;
    case msg.KEY_LEFT:
      goal_geom_.lin.p_.x() += d_lin;
      break;
    case msg.KEY_DOWN:
      goal_geom_.lin.p_.y() += d_lin;
      break;
    case msg.KEY_UP:
      goal_geom_.lin.p_.y() -= d_lin;
      break;
    case msg.KEY_PAGEUP:
      goal_geom_.lin.p_.z() += 0.5*d_lin;
      break;
    case msg.KEY_PAGEDOWN:
      goal_geom_.lin.p_.z() -= 0.5*d_lin;
      break;

    // desired goal orientations
    case msg.KEY_KP4:
      goal_geom_.ang.p_.x() -= d_ang; // roll-
      break;
    case msg.KEY_KP6:
      goal_geom_.ang.p_.x() += d_ang; // roll+
      break;
    case msg.KEY_KP8:
      goal_geom_.ang.p_.y() += d_ang; // pitch+
      break;
    case msg.KEY_KP2:
      goal_geom_.ang.p_.y() -= d_ang; // pitch-
      break;
    case msg.KEY_KP1:
      goal_geom_.ang.p_.z() += d_ang; // yaw+
      break;
    case msg.KEY_KP9:
      goal_geom_.ang.p_.z() -= d_ang; // yaw-
      break;

    // terrains
    case msg.KEY_1:
      terrain_id_ = terrain_id_<opt::HeightMap::K_TERRAIN_COUNT-1? terrain_id_+1 : 0;
      break;


    // speed
    case msg.KEY_KP_PLUS:
      total_duration_ += 0.2;
      ROS_INFO_STREAM("Total duration increased to " << total_duration_);
    break;
    case msg.KEY_KP_MINUS:
      total_duration_ -= 0.2;
      ROS_INFO_STREAM("Total duration decreased to " << total_duration_);
    break;


    case msg.KEY_g:
      ROS_INFO_STREAM("Request to optimize sent");
      optimize_ = true;
      break;
    case msg.KEY_w:
      ROS_INFO_STREAM("Motion type set to Walking");
      gait_type_ = quad::QuadrupedGaitGenerator::Walk;
      break;
    case msg.KEY_t:
      ROS_INFO_STREAM("Motion type set to Trotting");
      gait_type_ = quad::QuadrupedGaitGenerator::Trot;
      break;
    case msg.KEY_b:
      ROS_INFO_STREAM("Motion type set to Bounding");
      gait_type_ = quad::QuadrupedGaitGenerator::Bound;
      break;
    case msg.KEY_p:
      ROS_INFO_STREAM("Motion type set to Pace");
      gait_type_ = quad::QuadrupedGaitGenerator::Pace;
      break;
    case msg.KEY_s:
      ROS_INFO_STREAM("Toggled NLP solver type");
      use_solver_snopt_ = !use_solver_snopt_;
      break;
    case msg.KEY_r:
      ROS_INFO_STREAM("Replaying already optimized trajectory");
      replay_trajectory_ = true;
      break;
    default:
      break;
  }


  PublishCommand();
}




void NlpUserInputNode::PublishCommand()
{
//  if (!joy_msg_.axes.empty())
//    ModifyGoalJoy();

  xpp_msgs::UserCommand msg;
  msg.goal_lin          = RosConversions::XppToRos(goal_geom_.lin);
  msg.goal_ang          = RosConversions::XppToRos(goal_geom_.ang);
  msg.replay_trajectory = replay_trajectory_;
  msg.use_solver_snopt  = use_solver_snopt_;
  msg.optimize          = optimize_;
  msg.terrain_id        = terrain_id_;
  msg.gait_id           = gait_type_;
  msg.total_duration    = total_duration_;


  user_command_pub_.publish(msg);

  optimize_ = false;
  replay_trajectory_  = false;
}

//void
//NlpUserInputNode::ModifyGoalJoy ()
//{
//  enum Axis {L_LEFT = 0, L_FORWARD, R_LEFT, R_FORWARD};
//
//  double max_vel = 0.2; // [m/s]
//
//  double vel_y = -max_vel*joy_msg_.axes[L_FORWARD];
//  double vel_x = max_vel*joy_msg_.axes[L_LEFT];
//
//  // integrate velocity
//  double joy_deadzone = 0.05;
//  goal_geom_.lin.p_.x() += vel_x * joy_deadzone;
//  goal_geom_.lin.p_.y() += vel_y * joy_deadzone;
//}
//
//void
//NlpUserInputNode::CallbackJoy (const JoyMsg& msg)
//{
//  joy_msg_ = msg;
//
//  enum JoyButtons {X=0, A, B, Y};
//
//  if (joy_msg_.buttons[Y] == 1) {
//    command_ = Command::kStartWalking;
//    replay_trajectory_ = true;
//  }
//
//  if (joy_msg_.buttons[A] == 1) {
////    motion_type_ = opt::WalkID;
//  }
//
//  if (joy_msg_.buttons[X] == 1) {
////    motion_type_ = opt::TrotID;
//  }
//
//  PublishCommand();
//}

} /* namespace ros */
} /* namespace xpp */
