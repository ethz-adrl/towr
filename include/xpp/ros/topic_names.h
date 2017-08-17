/**
@file    topic_names.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 5, 2016
@brief   Brief description
 */

#ifndef XPP_MSGS_INCLUDE_XPP_MSGS_TOPIC_NAMES_H_
#define XPP_MSGS_INCLUDE_XPP_MSGS_TOPIC_NAMES_H_

#include <string>

namespace xpp_msgs {

// command that tells the walking controller to start executing
static const std::string start_walking_topic("xpp/start_walking");

// position of the desired goal to reach
static const std::string goal_state_topic("xpp/goal_state");

// the robot state sent from actual robot/simulation(SL).
static const std::string curr_robot_state_real("current_robot_state_real");

// the complete robot state including base, joints, time, ...
static const std::string curr_robot_state("current_robot_state");

// the current position of the robot as a ros pose marker
static const std::string curr_base_pose("current_base_pose");

// initial velocity (force vector)
static const std::string init_velocity("initial_velocity");

// initial velocity (force vector)
static const std::string opt_parameters("opt_parameters");

// sequence of cartesian robot states
static const std::string robot_trajectory_cart("hyq_cart_state_trajectory");

// sequence of full body hyq states as reference for a walking controller
static const std::string robot_trajectory_joints("hyq_joint_state_trajectory");

// sequence of full body hyq states as reference for a walking controller
static const std::string contact_vector("contact_vector");

// rviz marker topic for NLP's optimized variables (spline coefficients, footholds,...)
static const std::string rviz_optimized("/optimization_variables");

// rviz marker topic for NLP's optimized variables (spline coefficients, footholds,...)
static const std::string rviz_optimized_single("/optimization_variables_single");

// rviz marker for displaying the desired goal state
static const std::string goal_axis_rviz("/goal_axis_rviz");

// gains for hyq
static const std::string controller_pd_gains("hyq_pd_gains");
}

#endif /* XPP_MSGS_INCLUDE_XPP_MSGS_TOPIC_NAMES_H_ */
