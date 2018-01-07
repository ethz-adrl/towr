/*
 * topic_names.h
 *
 *  Created on: Jan 3, 2018
 *      Author: winklera
 */

#ifndef XPP_OPT_XPP_OPT_ROS_INCLUDE_XPP_OPT_ROS_TOPIC_NAMES_H_
#define XPP_OPT_XPP_OPT_ROS_INCLUDE_XPP_OPT_ROS_TOPIC_NAMES_H_

#include <string>

namespace xpp_msgs {

// position of the desired goal to reach
static const std::string user_command("/xpp/user_command");

// iterations the nlp took to solve the problem. Used when processing rosbags
static const std::string nlp_iterations_count("/xpp/nlp_iterations_count");

// the base topic names of each nlp iteration
static const std::string nlp_iterations_name("/xpp/nlp_iterations_name");
}


#endif /* XPP_OPT_XPP_OPT_ROS_INCLUDE_XPP_OPT_ROS_TOPIC_NAMES_H_ */
