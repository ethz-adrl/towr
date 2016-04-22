/*
 * xpp_optimizer_node.cc
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#include <xpp/ros/qp_optimizer_node.h>


namespace xpp {
namespace ros {

QpOptimizerNode::QpOptimizerNode ()
{
//  service_ = n.advertiseService("optimize_trajectory",
//                                &QpOptimizerNode::OptimizeTrajectoryService, this);
//  return_trajectory_service_ = n.advertiseService("return_optimized_trajectory",
//                                &QpOptimizerNode::ReturnOptimizedTrajectory, this);

}





//bool
//QpOptimizerNode::ReturnOptimizedTrajectory(xpp_opt::ReturnOptimizedTrajectory::Request& req,
//                               xpp_opt::ReturnOptimizedTrajectory::Response& res)
//{
//  res.x = xpp::ros::RosHelpers::XppToRos(opt_coefficients_, opt_footholds_);
//  return true;
//}
//
//
//
//bool
//QpOptimizerNode::OptimizeTrajectoryService(xpp_opt::OptimizeTrajectory::Request& req,
//                                            xpp_opt::OptimizeTrajectory::Response& res)
//{
//  goal_cog_ = StateLinMsgTo2DState(req.goal_state);
//  OptimizeTrajectory(opt_coefficients_, opt_footholds_);
//  return true;
//}


void
QpOptimizerNode::OptimizeTrajectory(VectorXd& opt_coefficients) const
{
//  qp_optimizer_.SolveQp(curr_cog_,
//                          goal_cog_,
//                          step_sequence,
//                          curr_stance_,
//                          opt_coefficients,
//                          opt_footholds);
}





} /* namespace ros */
} /* namespace xpp */
