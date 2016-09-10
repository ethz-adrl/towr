/*
 * optimizer_node_base.cc
 *
 *  Created on: Apr 22, 2016
 *      Author: winklera
 */

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp_opt/GetStateLin3d.h>

namespace xpp {
namespace ros {

using GetStateSrv = xpp_opt::GetStateLin3d;

OptimizerNodeBase::OptimizerNodeBase ()
{
  ::ros::NodeHandle n;

  // motion_ref do i actually need this? not doing anything
  goal_state_sub_ = n.subscribe("goal_state", 1,
                                &OptimizerNodeBase::GoalStateCallback, this);

  goal_state_client_ = n.serviceClient<GetStateSrv>("get_goal_state");

  t_swing_   = RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  t_stance_   = RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");

  robot_height_ = RosHelpers::GetDoubleFromServer("/xpp/robot_height");

  if (!CallService()) {
    // Get Starting goal state from server if nothing published yet
    goal_cog_.p.x() = RosHelpers::GetDoubleFromServer("/xpp/goal_state_x");
    goal_cog_.p.y() = RosHelpers::GetDoubleFromServer("/xpp/goal_state_y");
  }
}

bool
OptimizerNodeBase::CallService ()
{
  GetStateSrv srv;
  if (goal_state_client_.call(srv)) {
    goal_cog_ = RosHelpers::RosToXpp(srv.response.state);
    return true;
  }

  std::cout << "service failed";

  return false;
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
  PublishOptimizedValues();
}

} /* namespace ros */
} /* namespace xpp */

