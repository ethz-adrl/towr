/*
 * xpp_optimizer_node.cc
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */


#include <xpp/ros/nlp_optimizer_node.h>
#include <xpp/ros/ros_helpers.h>


namespace xpp {
namespace ros {

NlpOptimizerNode::NlpOptimizerNode ()
{
  optimize_trajectory_srv_ = n_.advertiseService("solve_nlp",
                                &NlpOptimizerNode::OptimizeTrajectoryService, this);
}



bool
NlpOptimizerNode::OptimizeTrajectoryService(xpp_opt::SolveNlp::Request& req,
                                            xpp_opt::SolveNlp::Response& res)
{
  curr_cog_ = RosHelpers::RosToXpp(req.curr_state);
  curr_stance_ = RosHelpers::RosToXpp(req.curr_stance);

  OptimizeTrajectory();
  return true;
}


void
NlpOptimizerNode::OptimizeTrajectory()
{
  std::vector<xpp::hyq::LegID> step_sequence = DetermineStepSequence();

  nlp_optimizer_.SolveNlp(curr_cog_,
                          goal_cog_,
                          step_sequence,
                          curr_stance_,
                          opt_splines_,
                          opt_footholds_);
}


std::vector<xpp::hyq::LegID>
NlpOptimizerNode::DetermineStepSequence() const
{
  const double length_per_step = 0.25;
  const double width_per_step = 0.15;
  Eigen::Vector2d start_to_goal = goal_cog_.p.segment<2>(0) - curr_cog_.p.segment<2>(0);

  int req_steps_by_length = std::ceil(std::fabs(start_to_goal.x())/length_per_step);
  int req_steps_by_width  = std::ceil(std::fabs(start_to_goal.y())/width_per_step);
  // get greatest value of all
  int req_steps_per_leg = std::max(req_steps_by_length,req_steps_by_width);

  using namespace xpp::hyq;
  const std::vector<xpp::hyq::LegID> take_4_steps = {LH, LF, RH, RF};
  std::vector<xpp::hyq::LegID> step_sequence;
  for (int i=0; i<req_steps_per_leg; ++i) {
    // insert 4 steps
    step_sequence.insert(step_sequence.end(), take_4_steps.begin(), take_4_steps.end());
  }

  return step_sequence;
}




} /* namespace ros */
} /* namespace xpp */
