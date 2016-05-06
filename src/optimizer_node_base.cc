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
//  opt_var_pub_ = n_.advertise<xpp_opt::OptimizedVariables>("opt_variables",10);
//  curr_state_sub_ = n_.subscribe("curr_state", 10,
//                                &OptimizerNodeBase::CurrentStateCallback, this);
  goal_state_sub_ = n_.subscribe("goal_state", 10,
                                &OptimizerNodeBase::GoalStateCallback, this);

  return_splines_srv_ = n_.advertiseService("return_optimized_splines",
                                &OptimizerNodeBase::ReturnOptimizedSplines, this);

  return_footholds_srv_ = n_.advertiseService("return_optimized_footholds",
                                &OptimizerNodeBase::ReturnOptimizedFootholds, this);


  double t_stance  = RosHelpers::GetDoubleFromServer("/xpp/stance_time");
  double t_swing   = RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  double t_init    = RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");
  double t_final   = RosHelpers::GetDoubleFromServer("/xpp/stance_time_final");
  spline_times_ = xpp::zmp::SplineTimes(t_stance, t_swing, t_init, t_final);

  robot_height_ = RosHelpers::GetDoubleFromServer("/xpp/robot_height");

  // This should be overwritten by joystick
  goal_cog_.p.x() = 0.25;
}

OptimizerNodeBase::~OptimizerNodeBase ()
{
  // TODO Auto-generated destructor stub
}


void
OptimizerNodeBase::CurrentStateCallback(const StateMsg& msg)
{
  curr_cog_ = RosHelpers::RosToXpp(msg);
  ROS_INFO_STREAM("Current state set to:\n" << goal_cog_);
}


void
OptimizerNodeBase::GoalStateCallback(const StateMsg& msg)
{
  goal_cog_ = RosHelpers::RosToXpp(msg);
  ROS_INFO_STREAM("Goal state set to:\n" << goal_cog_);
}


bool
OptimizerNodeBase::ReturnOptimizedSplines(ReturnOptSplinesSrv::Request& req,
                                          ReturnOptSplinesSrv::Response& res)
{
  res.splines = xpp::ros::RosHelpers::XppToRos(opt_splines_);
  return true;
}


bool
OptimizerNodeBase::ReturnOptimizedFootholds(ReturnOptFootholds::Request& req,
                                            ReturnOptFootholds::Response& res)
{
  res.footholds = xpp::ros::RosHelpers::XppToRos(footholds_);
  return true;
}



} /* namespace ros */
} /* namespace xpp */

