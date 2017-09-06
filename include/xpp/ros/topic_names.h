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

// the current robot cartesian state including base, feet, time, ...
static const std::string robot_state_current("xpp/state_curr");

// the desired state that comes from the optimizer
static const std::string robot_state_desired("xpp/state_des");

// sequence of desired states coming from the optimizer
static const std::string robot_trajectory_desired("xpp/trajectory_des");

// the fixed parameters used for the optimization
static const std::string opt_parameters("xpp/params");

// position of the desired goal to reach
static const std::string user_command("xpp/user_command");

//// command that tells the walking controller to start executing
//static const std::string start_walking_topic("xpp/start_walking");

// the robot state sent from actual robot/simulation(SL).
//static const std::string curr_robot_state_real("xpp/state_real");


// iterations the nlp took to solve the problem. Used when processing rosbags
static const std::string nlp_iterations_count("xpp/nlp_iterations_count");

// the base topic names of each nlp iteration
static const std::string nlp_iterations_name("xpp/nlp_iterations_name");


//// the current position of the robot as a ros pose marker
//static const std::string curr_base_pose("current_base_pose");
//
//// initial velocity (force vector)
//static const std::string init_velocity("initial_velocity");
//
//
//// sequence of full body hyq states as reference for a walking controller
//static const std::string robot_trajectory_joints("hyq_joint_state_trajectory");
//
//// sequence of full body hyq states as reference for a walking controller
//static const std::string contact_vector("contact_vector");
//
//
//// rviz marker topic for NLP's optimized variables (spline coefficients, footholds,...)
//static const std::string rviz_optimized_single("/optimization_variables_single");
//
//
//// gains for hyq
//static const std::string controller_pd_gains("hyq_pd_gains");
}

#endif /* XPP_MSGS_INCLUDE_XPP_MSGS_TOPIC_NAMES_H_ */
