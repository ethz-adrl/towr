/*
 * optimizer_node_base.cc
 *
 *  Created on: Apr 22, 2016
 *      Author: winklera
 */

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>
#include <xpp_msgs/GetStateLin3d.h>

namespace xpp {
namespace ros {

OptimizerNodeBase::OptimizerNodeBase ()
{
  ::ros::NodeHandle n;

  goal_state_sub_ = n.subscribe(xpp_msgs::goal_state_topic, 1,
                                &OptimizerNodeBase::GoalStateCallback, this);

  t_swing_   = RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  t_stance_initial_   = RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");

  des_robot_height_ = RosHelpers::GetDoubleFromServer("/xpp/robot_height");
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

  OptimizeTrajectory();
  PublishTrajectory();
}

} /* namespace ros */
} /* namespace xpp */

