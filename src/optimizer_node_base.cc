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
  goal_state_sub_ = n_.subscribe("goal_state", 1,
                                &OptimizerNodeBase::GoalStateCallback, this);

  goal_key_sub_ = n_.subscribe("/keyboard/keydown", 1,
                                &OptimizerNodeBase::GoalStateCallbackKeyboard, this);

  double t_swing   = RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  double t_init    = RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");
  spline_times_ = xpp::zmp::SplineTimes(t_swing, t_init);

  robot_height_ = RosHelpers::GetDoubleFromServer("/xpp/robot_height");

  // These should be overwritten by joystick
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


void
OptimizerNodeBase::GoalStateCallbackKeyboard(const keyboard::Key& msg)
{
  const static double dx = 0.1;
  const static double dy = 0.1;

  switch (msg.code) {
    case msg.KEY_UP:
      goal_cog_.p.x() = curr_cog_.p.x() + dx;
      break;
    case msg.KEY_DOWN:
      goal_cog_.p.x() = curr_cog_.p.x() - dx;
      break;
    case msg.KEY_RIGHT:
      goal_cog_.p.y() = curr_cog_.p.y() - dy;
      break;
    case msg.KEY_LEFT:
      goal_cog_.p.y() = curr_cog_.p.y() + dy;
      break;
    default:
      break;
  }

  ROS_INFO_STREAM("Goal state set to : " << goal_cog_);
}



} /* namespace ros */
} /* namespace xpp */

