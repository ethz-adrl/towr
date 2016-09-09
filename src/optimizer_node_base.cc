/*
 * optimizer_node_base.cc
 *
 *  Created on: Apr 22, 2016
 *      Author: winklera
 */

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/ros/ros_helpers.h>


namespace xpp {
namespace ros {

OptimizerNodeBase::OptimizerNodeBase ()
{
  ::ros::NodeHandle n_;

  // motion_ref do i actually need this? not doing anything
  goal_state_sub_ = n_.subscribe("goal_state", 1,
                                &OptimizerNodeBase::GoalStateCallback, this);

  t_swing_   = RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  t_stance_   = RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");

  robot_height_ = RosHelpers::GetDoubleFromServer("/xpp/robot_height");

  // Get Starting goal state from server if nothing published yet
  goal_cog_.p.x() = RosHelpers::GetDoubleFromServer("/xpp/goal_state_x");
  goal_cog_.p.y() = RosHelpers::GetDoubleFromServer("/xpp/goal_state_y");
}


OptimizerNodeBase::~OptimizerNodeBase ()
{
  // TODO Auto-generated destructor stub
}


void
OptimizerNodeBase::GoalStateCallback(const StateMsg& msg)
{
  goal_cog_ = RosHelpers::RosToXpp(msg);
  ROS_INFO_STREAM("Goal state set to:\n" << goal_cog_);
}

} /* namespace ros */
} /* namespace xpp */

