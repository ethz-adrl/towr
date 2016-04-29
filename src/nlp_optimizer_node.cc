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
  curr_cog_ = RosHelpers::RosToXpp(msg.curr_state);
  curr_stance_ = RosHelpers::RosToXpp(msg.curr_stance);

  OptimizeTrajectory();

  // send something out everytime something has been optimized
  OptParamMsg msg_out;
  msg_out.splines = xpp::ros::RosHelpers::XppToRos(opt_splines_);
  msg_out.footholds = xpp::ros::RosHelpers::XppToRos(footholds_);
  opt_params_pub_.publish(msg_out);
}


void
NlpOptimizerNode::OptimizeTrajectory()
{
  std::vector<xpp::hyq::LegID> step_sequence = DetermineStepSequence();

  double swing_time          = RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  double stance_time         = RosHelpers::GetDoubleFromServer("/xpp/stance_time");
  double stance_time_initial = RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");
  double stance_time_final   = RosHelpers::GetDoubleFromServer("/xpp/stance_time_final");
  double robot_height        = RosHelpers::GetDoubleFromServer("/xpp/robot_height");


  nlp_optimizer_.SolveNlp(curr_cog_.Get2D(),
                          goal_cog_.Get2D(),
                          step_sequence,
                          curr_stance_,
                          swing_time,
                          stance_time,
                          stance_time_initial,
                          stance_time_final,
                          robot_height,
                          opt_splines_,
                          footholds_);
}


std::vector<xpp::hyq::LegID>
NlpOptimizerNode::DetermineStepSequence() const
{
  const double length_per_step = 0.35;
  const double width_per_step = 0.20;
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
