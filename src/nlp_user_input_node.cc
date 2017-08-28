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

namespace xpp {
namespace ros {

using UserCommandMsg = xpp_msgs::UserCommand;

NlpUserInputNode::NlpUserInputNode ()
{
  ::ros::NodeHandle n;
  key_sub_ = n.subscribe("/keyboard/keydown", 1, &NlpUserInputNode::CallbackKeyboard, this);
  joy_sub_ = n.subscribe("/joy", 1, &NlpUserInputNode::CallbackJoy, this);

  user_command_pub_ = n.advertise<UserCommandMsg>(xpp_msgs::user_command, 1);

  // publish goal zero initially
  goal_geom_.lin.p_.setZero();
  goal_geom_.lin.p_ << 0.5, 0.0, 0.58;
  goal_geom_.ang.p_ << 0.0, 0.0, 0.0; // roll, pitch, yaw angle applied Z->Y'->X''
  replay_trajectory_ = false;
  use_solver_snopt_ = false;


  terrain_id_ = opt::HeightMap::FlatID;

//  UserCommandMsg msg;
//  msg.goal_lin = RosConversions::XppToRos(goal_geom_.lin);
//  msg.goal_ang = RosConversions::XppToRos(goal_geom_.ang);
//  user_command_pub_.publish(msg);
//
//  // start walking command
//  walk_command_pub_ = n.advertise<std_msgs::Empty>(xpp_msgs::start_walking_topic,1);
}

NlpUserInputNode::~NlpUserInputNode ()
{
  // TODO Auto-generated destructor stub
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



    case msg.KEY_RETURN:
      command_ = Command::kSetGoal;
      break;
    case msg.KEY_g:
      ROS_INFO_STREAM("Start Walking command sent to controller");
      command_ = Command::kStartWalking;
      break;
    case msg.KEY_w:
      ROS_INFO_STREAM("Motion type set to Walking");
//      motion_type_ = opt::WalkID;
      break;
    case msg.KEY_t:
      ROS_INFO_STREAM("Motion type set to Trotting");
//      motion_type_ = opt::TrotID;
      break;
    case msg.KEY_b:
      ROS_INFO_STREAM("Motion type set to Bounding");
//      motion_type_ = opt::BoundID;
      break;
    case msg.KEY_c:
      ROS_INFO_STREAM("Motion type set to Camel");
//      motion_type_ = opt::PaceID;
      break;
    case msg.KEY_s:
      ROS_INFO_STREAM("Toggled NLP solver type");
      use_solver_snopt_ = !use_solver_snopt_;
      break;
    case msg.KEY_p:
      ROS_INFO_STREAM("Motion type set to Push Recovery");
//      motion_type_ = opt::PushRecID;
      break;
    case msg.KEY_r:
      ROS_INFO_STREAM("Replaying already optimized trajectory");
      replay_trajectory_ = true;
      break;
    case msg.KEY_d: {

      std::vector<double> x_vel = { 1.3,   1.4,  0.1,  -1.4, -0.8,  0.8,  1.9   };
      std::vector<double> y_vel = { -0.4,  0.05, 1.5,   0.0, -1.5,  1.5,  0.0   };

      static int idx = 0;
      velocity_disturbance_.x = x_vel.at(idx);
      velocity_disturbance_.y = y_vel.at(idx);
      velocity_disturbance_.z = 0.0;

      idx = idx<x_vel.size()-1 ? idx+1 : 0; // circular buffer
      ROS_INFO_STREAM("Added initial velocity disturbance of (" << x_vel.at(idx) << "," << y_vel.at(idx) << ")." );

    }
    default:
      break;
  }


  PublishCommand();
}

void
NlpUserInputNode::CallbackJoy (const JoyMsg& msg)
{
  joy_msg_ = msg;

  enum JoyButtons {X=0, A, B, Y};

  if (joy_msg_.buttons[Y] == 1) {
    command_ = Command::kStartWalking;
    replay_trajectory_ = true;
  }

  if (joy_msg_.buttons[A] == 1) {
//    motion_type_ = opt::WalkID;
  }

  if (joy_msg_.buttons[X] == 1) {
//    motion_type_ = opt::TrotID;
  }

  PublishCommand();
}

void
NlpUserInputNode::ModifyGoalJoy ()
{
  enum Axis {L_LEFT = 0, L_FORWARD, R_LEFT, R_FORWARD};

  double max_vel = 0.2; // [m/s]

  double vel_y = -max_vel*joy_msg_.axes[L_FORWARD];
  double vel_x = max_vel*joy_msg_.axes[L_LEFT];

  // integrate velocity
  double joy_deadzone = 0.05;
  goal_geom_.lin.p_.x() += vel_x * joy_deadzone;
  goal_geom_.lin.p_.y() += vel_y * joy_deadzone;
}

void NlpUserInputNode::PublishCommand()
{
  if (!joy_msg_.axes.empty())
    ModifyGoalJoy();

  UserCommandMsg msg;
  msg.goal_lin          = RosConversions::XppToRos(goal_geom_.lin);
  msg.goal_ang          = RosConversions::XppToRos(goal_geom_.ang);
  msg.replay_trajectory = replay_trajectory_;
  msg.use_solver_snopt  = use_solver_snopt_;
  msg.vel_disturbance   = velocity_disturbance_;
  msg.optimize          = command_==Command::kStartWalking? true : false;
  msg.terrain_id        = terrain_id_;
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

  command_ = Command::kNoCommand;
  replay_trajectory_  = false;
  velocity_disturbance_ = InitVel(); // set zero
}


} /* namespace ros */
} /* namespace xpp */
