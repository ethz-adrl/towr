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
  opt_var_pub_ = n_.advertise<xpp_opt::OptimizedVariables>("opt_variables",10);
  curr_state_sub_ = n_.subscribe("curr_state", 10,
                                &OptimizerNodeBase::CurrentStateCallback, this);
  goal_state_sub_ = n_.subscribe("goal_state", 10,
                                &OptimizerNodeBase::GoalStateCallback, this);

  //fixme get this from robot
  using namespace xpp::hyq;
  curr_stance_[LF] = Foothold( 0.35,  0.3, 0.0, LF);
  curr_stance_[RF] = Foothold( 0.35, -0.3, 0.0, RF);
  curr_stance_[LH] = Foothold(-0.35,  0.3, 0.0, LH);
  curr_stance_[RH] = Foothold(-0.35, -0.3, 0.0, RH);
}

OptimizerNodeBase::~OptimizerNodeBase ()
{
  // TODO Auto-generated destructor stub
}


void
OptimizerNodeBase::CurrentStateCallback(const StateMsg& msg)
{
  curr_cog_ = RosHelpers::StateLinMsgTo2DState(msg);
}


void
OptimizerNodeBase::GoalStateCallback(const StateMsg& msg)
{
  goal_cog_ = RosHelpers::StateLinMsgTo2DState(msg);
}


} /* namespace ros */
} /* namespace xpp */

