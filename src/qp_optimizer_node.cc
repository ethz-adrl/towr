/*
 * xpp_optimizer_node.cc
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/qp_optimizer_node.h>

namespace xpp {
namespace ros {

QpOptimizerNode::QpOptimizerNode ()
{
  current_info_sub_ = n_.subscribe("required_info_qp",
                                   1, // take only the most recent information
                                   &QpOptimizerNode::CurrentInfoCallback, this);

  opt_params_pub_ = n_.advertise<OptParamMsg>("optimized_parameters_qp", 1);
}

void
QpOptimizerNode::CurrentInfoCallback(const ReqInfoMsg& msg)
{
  // fixme DRY: use template method to move this and qp code to base class
  UpdateCurrentState(msg);
  OptimizeTrajectory();
  PublishOptimizedValues();
}

void
QpOptimizerNode::UpdateCurrentState(const ReqInfoMsg& msg)
{
  curr_cog_    = RosHelpers::RosToXpp(msg.curr_state);
  curr_stance_ = RosHelpers::RosToXpp(msg.curr_stance);
  footholds_   = RosHelpers::RosToXpp(msg.steps);
  start_with_com_shift_ = msg.start_with_com_shift;
}

void
QpOptimizerNode::PublishOptimizedValues() const
{
  OptParamMsg msg_out;
  msg_out.splines = xpp::ros::opt::RosHelpers::XppToRos(opt_splines_);
  opt_params_pub_.publish(msg_out);
}

void
QpOptimizerNode::OptimizeTrajectory()
{
  opt_splines_ = qp_optimizer_.SolveQp(curr_cog_.Get2D(),
                                       goal_cog_.Get2D(),
                                       curr_stance_,
                                       footholds_,
                                       t_swing_, t_stance_,
                                       start_with_com_shift_,
                                       robot_height_);
}

} /* namespace ros */
} /* namespace xpp */
