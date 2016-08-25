/**
 @file    optimization_visualizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Brief description
 */

#include <xpp/ros/optimization_visualizer.h>
#include <xpp/ros/ros_helpers.h>

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

  std::vector<xpp::zmp::ComPolynomial> spline   = observer_->GetSplines();
  std::vector<xpp::hyq::Foothold> footholds = observer_->GetFootholds();
  std::vector<xpp::hyq::Foothold> start_stance = observer_->GetStartStance();

  visualization_msgs::MarkerArray msg;
  msg_builder_.AddStartStance(msg, start_stance);
  msg_builder_.AddFootholds(msg, footholds, "footholds", visualization_msgs::Marker::CUBE, 1.0);
  msg_builder_.AddCogTrajectory(msg, spline, footholds, "cog", 1.0);
  msg_builder_.AddZmpTrajectory(msg, spline, walking_height, footholds, "zmp_4ls", 0.7);
  msg_builder_.AddSupportPolygons(msg, start_stance, footholds);
//  msg_builder_.AddLineStrip(msg, -0.2, 0.2, "gap");
//  msg_builder_.AddEllipse(msg, -0.2, 0.0, 0.15, 2.0, "ellipse");

//  static int n_markers_first_iteration = msg.markers.size();
//  for (int i=n_markers_first_iteration; i<msg.markers.size(); ++i) {
//    msg.markers.at(i).type = visualization_msgs::Marker::DELETE;
//    msg.markers.at(i).color.a = 0.0;
//  }

  ros_publisher_.publish(msg);
}

} /* namespace ros */
} /* namespace xpp */
