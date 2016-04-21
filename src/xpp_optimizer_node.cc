/*
 * xpp_optimizer_node.cc
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#include <xpp/ros/xpp_optimizer_node.h>
#include <xpp_opt/OptimizedVariables.h>

namespace xpp {
namespace ros {

XppOptimizerNode::XppOptimizerNode ()
{
  // TODO Auto-generated constructor stub
  ::ros::NodeHandle n;
  opt_var_pub_ = n.advertise<xpp_opt::OptimizedVariables>("opt_variables",10);
  curr_state_sub_ = n.subscribe("curr_state", 10, &XppOptimizerNode::CurrentStateCallback, this);
  goal_state_sub_ = n.subscribe("curr_state", 10, &XppOptimizerNode::GoalStateCallback, this);

}

XppOptimizerNode::~XppOptimizerNode ()
{
  // TODO Auto-generated destructor stub
}


void
XppOptimizerNode::OptimizeTrajectory() const
{

}


void
XppOptimizerNode::CurrentStateCallback(const xpp_opt::StateLin3d& msg)
{
  curr_ = StateLinMsgToPoint(msg);
}


void
XppOptimizerNode::GoalStateCallback(const xpp_opt::StateLin3d& msg)
{
  goal_ = StateLinMsgToPoint(msg);
}


XppOptimizerNode::Point3d
XppOptimizerNode::StateLinMsgToPoint(const xpp_opt::StateLin3d& msg) const
{
  Point3d point;
  point.p.x() = msg.pos.x;
  point.p.y() = msg.pos.y;
  point.p.z() = msg.pos.z;

  point.v.x() = msg.vel.x;
  point.v.y() = msg.vel.y;
  point.v.z() = msg.vel.z;

  point.a.x() = msg.acc.x;
  point.a.y() = msg.acc.y;
  point.a.z() = msg.acc.z;

  return point;
}



} /* namespace ros */
} /* namespace xpp */
