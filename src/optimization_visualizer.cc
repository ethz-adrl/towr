/**
 @file    optimization_visualizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Brief description
 */

#include <xpp/ros/optimization_visualizer.h>
#include <xpp/ros/ros_helpers.h>

#include <xpp/zmp/zmp_spline.h>
#include <xpp/hyq/foothold.h>
#include <xpp/zmp/interpreting_observer.h>

namespace xpp {
namespace ros {

OptimizationVisualizer::OptimizationVisualizer ()
{
  observer_ = nullptr;

  ::ros::NodeHandle n;
  ros_publisher_ = n.advertise<visualization_msgs::MarkerArray>("optimization_variables", 1);
}

OptimizationVisualizer::~OptimizationVisualizer ()
{
}

void
OptimizationVisualizer::SetObserver (const InterpretingObserverPtr& observer)
{
  observer_ = observer;
}

void
OptimizationVisualizer::Visualize () const
{
  double walking_height = RosHelpers::GetDoubleFromServer("/xpp/robot_height");

  std::vector<xpp::zmp::ZmpSpline> spline   = observer_->GetSplines();
  std::vector<xpp::hyq::Foothold> footholds = observer_->GetFootholds();

  visualization_msgs::MarkerArray msg = msg_builder_.BuildMsg(spline, footholds, walking_height);
  ros_publisher_.publish(msg);
}

} /* namespace ros */
} /* namespace xpp */
