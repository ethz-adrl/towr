/**
 @file    optimization_visualizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Brief description
 */

#include <xpp/ros/optimization_visualizer.h>

#include <xpp/ros/ros_helpers.h>

namespace xpp {
namespace ros {

xpp::ros::OptimizationVisualizer::OptimizationVisualizer (
    OptimizationVariables& subject)
    : subject_(subject)
{
  subject_.RegisterObserver(this);

  ::ros::NodeHandle n;
  ros_publisher_ = n.advertise<visualization_msgs::MarkerArray>("optimization_variables", 1);
}

void
xpp::ros::OptimizationVisualizer::Update ()
{
  x_coeff_ = subject_.GetSplineCoefficients();
  footholds_xy_ = subject_.GetFootholdsStd();
}

void
OptimizationVisualizer::Init (const std::vector<LegID>& swing_leg_sequence,
                              const ContinuousSplineContainer& splines)
{
  leg_ids_ = swing_leg_sequence;
  splines_ = splines;
}

void
xpp::ros::OptimizationVisualizer::PublishMsg ()
{
  // fill in some semantic information to interpret optimization variables
  // fixme, this replicates NlpOptimizer::GetFootholds
  splines_.AddOptimizedCoefficients(x_coeff_);
  VecSpline splines = splines_.GetSplines();

  VecFoothold footholds(footholds_xy_.size());
  for (uint i=0; i<footholds.size(); ++i) {
    footholds.at(i).leg = leg_ids_.at(i);
    footholds.at(i).p.x() = footholds_xy_.at(i).x();
    footholds.at(i).p.y() = footholds_xy_.at(i).y();
  }

  double walking_height = RosHelpers::GetDoubleFromServer("/xpp/robot_height");

  visualization_msgs::MarkerArray msg = msg_builder_.BuildMsg(splines, footholds, walking_height);
  ros_publisher_.publish(msg);
}

} /* namespace ros */
} /* namespace xpp */
