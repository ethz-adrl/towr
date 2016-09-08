/**
 @file    optimization_visualizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Brief description
 */

#include <xpp/ros/optimization_visualizer.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/marker_array_builder.h>
#include "../include/xpp/zmp/nlp_observer.h"

namespace xpp {
namespace ros {

using VectorXd = Eigen::VectorXd;

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

  auto start_stance = observer_->GetStartStance();
  auto com_motion   = observer_->GetComMotion();
  auto footholds    = observer_->GetFootholds();
  auto structure    = observer_->GetStructure();

  visualization_msgs::MarkerArray msg;
  MarkerArrayBuilder msg_builder_;
  msg_builder_.AddStartStance(msg, start_stance);
  msg_builder_.AddFootholds(msg, footholds, "footholds", visualization_msgs::Marker::CUBE, 1.0);
  msg_builder_.AddCogTrajectory(msg, *com_motion, structure, footholds, "cog", 1.0);
  msg_builder_.AddZmpTrajectory(msg, *com_motion, structure, walking_height, footholds, "zmp_4ls", 0.2);
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
