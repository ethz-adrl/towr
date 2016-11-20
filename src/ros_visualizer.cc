/**
 @file    optimization_visualizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Brief description
 */

#include "../include/xpp/ros/ros_visualizer.h"

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/marker_array_builder.h>
#include <xpp/ros/topic_names.h>
#include <xpp/opt/nlp_observer.h>

namespace xpp {
namespace ros {

using VectorXd = Eigen::VectorXd;

RosVisualizer::RosVisualizer ()
{
  observer_ = nullptr;

  ::ros::NodeHandle n;
  ros_publisher_optimized_ = n.advertise<visualization_msgs::MarkerArray>(xpp_msgs::rviz_optimized, 1);
  ros_publisher_fixed_     = n.advertise<visualization_msgs::MarkerArray>(xpp_msgs::rviz_fixed, 1);
}

RosVisualizer::~RosVisualizer ()
{
//  visual_tools_->deleteAllMarkers();
}

void
RosVisualizer::VisualizeCurrentState (const State& curr,
                                               const VecFoothold& start_stance) const
{
  visualization_msgs::MarkerArray msg;
  MarkerArrayBuilder msg_builder;

  msg_builder.AddPoint(msg, curr.p, "current", visualization_msgs::Marker::CYLINDER);
  msg_builder.AddStartStance(msg, start_stance);

  ros_publisher_fixed_.publish(msg);
}

void
RosVisualizer::Visualize () const
{
  double walking_height = RosHelpers::GetDoubleFromServer("/xpp/robot_height");

  auto start_stance = observer_->GetStartStance();
  auto com_motion   = observer_->GetComMotion();
  auto footholds    = observer_->GetFootholds();
  auto structure    = observer_->GetStructure();

  visualization_msgs::MarkerArray msg;
  MarkerArrayBuilder msg_builder_;
//  msg_builder_.AddStartStance(msg, start_stance);
  msg_builder_.AddFootholds(msg, footholds, "footholds", visualization_msgs::Marker::CUBE, 1.0);
  msg_builder_.AddCogTrajectory(msg, *com_motion, structure, footholds, "cog", 1.0);
//  msg_builder_.AddZmpTrajectory(msg, *com_motion, structure, walking_height, footholds, "zmp_4ls", 0.2);
  msg_builder_.AddSupportPolygons(msg, start_stance, footholds);
//  msg_builder_.AddGoal(msg, goal_cog_.Get2D().p);
  double gap_center_x = 0.45;
  double gap_width_x = 0.2;
  double ellipse_width = 1.0;

//  msg_builder_.AddLineStrip(msg, gap_center_x, gap_width_x, "gap");
//  msg_builder_.AddEllipse(msg, gap_center_x, 0.0, gap_width_x, ellipse_width, "ellipse");

//  static int n_markers_first_iteration = msg.markers.size();
//  for (int i=n_markers_first_iteration; i<msg.markers.size(); ++i) {
//    msg.markers.at(i).type = visualization_msgs::Marker::DELETE;
//    msg.markers.at(i).color.a = 0.0;
//  }

  ros_publisher_optimized_.publish(msg);
}

} /* namespace ros */
} /* namespace xpp */
