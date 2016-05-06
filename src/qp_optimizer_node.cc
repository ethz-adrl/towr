/*
 * xpp_optimizer_node.cc
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#include <xpp/ros/qp_optimizer_node.h>
#include <xpp/ros/ros_helpers.h>


namespace xpp {
namespace ros {

QpOptimizerNode::QpOptimizerNode ()
{
  current_info_sub_ = n_.subscribe("required_info_qp",
                                   1, // take only the most recent information
                                   &QpOptimizerNode::CurrentInfoCallback, this);

  opt_params_pub_ = n_.advertise<OptParamMsg>("optimized_parameters_qp", 1);

  opt_srv_ = n_.advertiseService("optimize_trajectory",
                                &QpOptimizerNode::OptimizeTrajectoryService, this);
}


void
QpOptimizerNode::CurrentInfoCallback(const ReqInfoMsg& msg)
{
  curr_cog_    = RosHelpers::RosToXpp(msg.curr_state);
  curr_stance_ = RosHelpers::RosToXpp(msg.curr_stance);
  footholds_   = RosHelpers::RosToXpp(msg.steps);

  OptimizeTrajectory();

  // send something out everytime something has been optimized
  OptParamMsg msg_out;
  msg_out.splines = xpp::ros::RosHelpers::XppToRos(opt_splines_);
  opt_params_pub_.publish(msg_out);
}


bool
QpOptimizerNode::OptimizeTrajectoryService(xpp_opt::SolveQp::Request& req,
                                           xpp_opt::SolveQp::Response& res)
{
  curr_cog_ = RosHelpers::RosToXpp(req.curr_state);
  curr_stance_ = RosHelpers::RosToXpp(req.curr_stance);

  int n_steps = req.steps.size();
  footholds_.resize(n_steps);
  for (int i=0; i<n_steps; i++) {
    footholds_.at(i) = RosHelpers::RosToXpp(req.steps.at(i));
  }

  OptimizeTrajectory();
  return true;
}


void
QpOptimizerNode::OptimizeTrajectory()
{
  opt_splines_ = qp_optimizer_.SolveQp(curr_cog_.Get2D(),
                                       goal_cog_.Get2D(),
                                       curr_stance_,
                                       footholds_,
                                       spline_times_,
                                       robot_height_);
}





} /* namespace ros */
} /* namespace xpp */
