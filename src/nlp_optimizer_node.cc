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
  current_info_sub_ = n_.subscribe("required_info_nlp",
                                   1, // take only the most recent information
                                   &NlpOptimizerNode::CurrentInfoCallback, this);

  opt_params_pub_ = n_.advertise<OptParamMsg>("optimized_parameters_nlp", 1);
}


void
NlpOptimizerNode::CurrentInfoCallback(const ReqInfoMsg& msg)
{
  // fixme DRY: use template method to move this and qp code to base class
  UpdateCurrentState(msg);
  OptimizeTrajectory();
  PublishOptimizedValues();
}


void
NlpOptimizerNode::UpdateCurrentState(const ReqInfoMsg& msg)
{
  curr_cog_      = RosHelpers::RosToXpp(msg.curr_state);
  curr_stance_   = RosHelpers::RosToXpp(msg.curr_stance);
  curr_swingleg_ = RosHelpers::RosToXpp(msg.curr_swingleg);
}


void
NlpOptimizerNode::PublishOptimizedValues() const
{
  OptParamMsg msg_out;
  msg_out.splines   = xpp::ros::RosHelpers::XppToRos(opt_splines_);
  msg_out.footholds = xpp::ros::RosHelpers::XppToRos(footholds_);

  opt_params_pub_.publish(msg_out);
}


void
NlpOptimizerNode::OptimizeTrajectory()
{
  std::vector<xpp::hyq::LegID> step_sequence = DetermineStepSequence(curr_cog_, curr_swingleg_);
  std::cout << "step_sequence.size(): " << step_sequence.size() << std::endl;

  nlp_optimizer_.SolveNlp(curr_cog_.Get2D(),
                          goal_cog_.Get2D(),
                          step_sequence,
                          curr_stance_,
                          spline_times_,
                          robot_height_,
                          opt_splines_,
                          footholds_);
}


std::vector<xpp::hyq::LegID>
NlpOptimizerNode::DetermineStepSequence(const State& curr_state, LegID curr_swingleg) const
{
  // TODO make step sequence dependent on curr_state
  const double length_per_step = 0.30;
  const double width_per_step = 0.20;
  Eigen::Vector2d start_to_goal = goal_cog_.p.segment<2>(0) - curr_cog_.p.segment<2>(0);

  int req_steps_per_leg;
  // fixme don't take steps if body movement is sufficient
  if (false /*start_to_goal.norm() < 0.1*/) {
    req_steps_per_leg = 0;
  } else {
    int req_steps_by_length = std::ceil(std::fabs(start_to_goal.x())/length_per_step);
    int req_steps_by_width  = std::ceil(std::fabs(start_to_goal.y())/width_per_step);
    req_steps_per_leg = std::max(req_steps_by_length,req_steps_by_width);
  }


  using namespace xpp::hyq;
  std::vector<xpp::hyq::LegID> step_sequence;

  LegID curr = curr_swingleg;
  for (int step=0; step<req_steps_per_leg*4; ++step) {
    step_sequence.push_back(NextSwingLeg(curr));
    curr = step_sequence.back();
  }

  return step_sequence;
}


NlpOptimizerNode::LegID
NlpOptimizerNode::NextSwingLeg(LegID curr) const
{
  using namespace xpp::hyq;

  switch (curr) {
    case LH: return LF;
    case LF: return RH;
    case RH: return RF;
    case RF: return LH;
    default: assert(false); // this should never happen
  };
}




} /* namespace ros */
} /* namespace xpp */
