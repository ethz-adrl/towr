/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
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

  supp_polygon_margins_ = xpp::hyq::SupportPolygon::GetDefaultMargins();
  supp_polygon_margins_[hyq::DIAG] = RosHelpers::GetDoubleFromServer("/xpp/margin_diag");

  max_cpu_time_ = RosHelpers::GetDoubleFromServer("/xpp/max_cpu_time");

  // get current optimization values from the optimizer
  optimization_visualizer_.SetObserver(nlp_facade_.GetObserver());
  nlp_facade_.AttachVisualizer(optimization_visualizer_);
}

void
NlpOptimizerNode::CurrentInfoCallback(const ReqInfoMsg& msg)
{
  UpdateCurrentState(msg);
//  OptimizeTrajectory();
//  PublishOptimizedValues();
//  optimization_visualizer_.Visualize();
}

void
NlpOptimizerNode::UpdateCurrentState(const ReqInfoMsg& msg)
{
  curr_cog_      = RosHelpers::RosToXpp(msg.curr_state);
  curr_stance_   = RosHelpers::RosToXpp(msg.curr_stance);
  curr_swingleg_ = msg.curr_swingleg;
}

void
NlpOptimizerNode::PublishOptimizedValues() const
{
  OptParamMsg msg_out;
  msg_out.splines   = xpp::ros::RosHelpers::XppToRos(opt_splines_);
  msg_out.footholds = xpp::ros::RosHelpers::XppToRos(footholds_);
  msg_out.phases    = xpp::ros::RosHelpers::XppToRos(motion_phases_);

  opt_params_pub_.publish(msg_out);
  ROS_INFO_STREAM("Publishing optimized values");
}

void
NlpOptimizerNode::OptimizeTrajectory()
{
  nlp_facade_.SolveNlp(curr_cog_.Get2D(),
                       goal_cog_.Get2D(),
                       curr_swingleg_,
                       robot_height_,
                       curr_stance_,
                       supp_polygon_margins_,
                       t_swing_, t_stance_,
                       max_cpu_time_);

  auto& com_spline = dynamic_cast<xpp::zmp::ComSpline&>(*nlp_facade_.GetMotion());
  opt_splines_   = com_spline.GetPolynomials();
  footholds_     = nlp_facade_.GetFootholds();
  motion_phases_ = nlp_facade_.GetPhases();

  optimization_visualizer_.Visualize();
}

} /* namespace ros */
} /* namespace xpp */
