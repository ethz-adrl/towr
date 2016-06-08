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

OptimizationVisualizer::OptimizationVisualizer ()
{
  ::ros::NodeHandle n;
  ros_publisher_ = n.advertise<visualization_msgs::MarkerArray>("optimization_variables", 1);
}

void
OptimizationVisualizer::SetInterpreter (const InterpretingObserverPtr& interpreter)
{
  observer_ = interpreter;
}

void
OptimizationVisualizer::PublishMsg ()
{
  double walking_height = RosHelpers::GetDoubleFromServer("/xpp/robot_height");

  VecSpline spline      = observer_->GetSplines();
  VecFoothold footholds = observer_->GetFootholds();

  visualization_msgs::MarkerArray msg = msg_builder_.BuildMsg(spline, footholds, walking_height);
  ros_publisher_.publish(msg);
}

} /* namespace ros */
} /* namespace xpp */
