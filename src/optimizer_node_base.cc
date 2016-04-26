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

  return_coeff_srv_ = n_.advertiseService("return_optimized_coeff",
                                &OptimizerNodeBase::ReturnOptimizedCoeff, this);

  // FIXME, this should come from joystick
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
}


void
OptimizerNodeBase::GoalStateCallback(const StateMsg& msg)
{
  goal_cog_ = RosHelpers::RosToXpp(msg);
}


bool
OptimizerNodeBase::ReturnOptimizedCoeff(ReturnOptSplinesSrv::Request& req,
                                        ReturnOptSplinesSrv::Response& res)
{
  res.splines = xpp::ros::RosHelpers::XppToRos(opt_splines_);
  return true;
}


} /* namespace ros */
} /* namespace xpp */

