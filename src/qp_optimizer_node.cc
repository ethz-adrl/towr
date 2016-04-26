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
  opt_srv_ = n_.advertiseService("optimize_trajectory",
                                &QpOptimizerNode::OptimizeTrajectoryService, this);
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
  opt_splines_ = qp_optimizer_.SolveQp(curr_cog_,
                                       goal_cog_,
                                       curr_stance_,
                                       footholds_);
}





} /* namespace ros */
} /* namespace xpp */
