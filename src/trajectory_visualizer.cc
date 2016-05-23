/*
 * zmp_publisher.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: winklera
 */

#include <xpp/ros/ros_helpers.h>

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/zero_moment_point.h>
#include "../include/xpp/ros/trajectory_visualizer.h"


namespace xpp {
namespace ros {

TrajectoryVisualizer::TrajectoryVisualizer(const std::string& topic)
{
  zmp_msg_.markers.clear();

  ::ros::NodeHandle n;
  ros_publisher_ = n.advertise<visualization_msgs::MarkerArray>(topic, 1);

  walking_height_ = RosHelpers::GetDoubleFromServer("/xpp/robot_height");
}


void TrajectoryVisualizer::AddRvizMessage(
    const VecSpline& splines,
    const VecFoothold& opt_footholds,
    const VecFoothold& start_stance,
    double gap_center_x,
    double gap_width_x,
    double alpha)
{
  zmp_msg_.markers.clear();

  visualization_msgs::MarkerArray msg;
  AddFootholds(msg, opt_footholds, "footholds", visualization_msgs::Marker::CUBE, alpha);
  AddStartStance(msg, start_stance);
  AddCogTrajectory(msg, splines, opt_footholds, "cog", alpha);
  AddZmpTrajectory(msg, splines, opt_footholds, "zmp_4ls", 0.7);
  AddSupportPolygons(msg, start_stance, opt_footholds);

//  AddLineStrip(msg, gap_center_x, gap_width_x, "gap");

  zmp_msg_.markers.insert(zmp_msg_.markers.end(),
                          msg.markers.begin(),msg.markers.end());
}


void TrajectoryVisualizer::AddStartStance(visualization_msgs::MarkerArray& msg,
                                  const VecFoothold& start_stance)
{
  AddFootholds(msg, start_stance, "start_stance", visualization_msgs::Marker::SPHERE, 1.0);
}


void TrajectoryVisualizer::AddSupportPolygons(visualization_msgs::MarkerArray& msg,
                                      const VecFoothold& start_stance,
                                      const VecFoothold& footholds) const
{
  xpp::hyq::SupportPolygonContainer support_polygon_container;
  support_polygon_container.Init(start_stance, footholds);


  xpp::hyq::SupportPolygonContainer::VecSupportPolygon supp;
  supp = support_polygon_container.GetSupportPolygons();

  for (uint i=0; i<supp.size(); ++i) {
    visualization_msgs::Marker m = BuildSupportPolygon(supp.at(i).footholds_conv_, footholds.at(i).leg);
    msg.markers.push_back(m);
  }
}


visualization_msgs::Marker
TrajectoryVisualizer::BuildSupportPolygon(const VecFoothold& stance,
                                     xpp::hyq::LegID leg_id) const
{
//  static int i=0;
//  geometry_msgs::PolygonStamped polygon_msg;
//  polygon_msg.header.frame_id = "world";
//  polygon_msg.header.seq = i++;


  visualization_msgs::Marker marker;
  marker.id = (zmp_msg_.markers.size() == 0)? 0 : zmp_msg_.markers.back().id + 1;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ::ros::Time();
  marker.ns = "leg " + std::to_string(leg_id);
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
 //    marker.lifetime = ros::Duration(10);
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  marker.color = GetLegColor(leg_id);
  marker.color.a = 0.1;

  geometry_msgs::Point p1;
  for (size_t i=0; i<stance.size(); ++i) {
    p1.x = stance.at(i).p.x();
    p1.y = stance.at(i).p.y();
    p1.z = stance.at(i).p.z();
    marker.points.push_back(p1);
  }

  return marker;


//  geometry_msgs::Point32 p1;
//  for (size_t i=0; i<footholds.size(); ++i) {
//    p1.x = footholds.at(i).p.x();
//    p1.y = footholds.at(i).p.y();
//    p1.z = footholds.at(i).p.z();
//    polygon_msg.polygon.points.push_back(p1);
//  }
//  ros__publisher_.publish(polygon_msg);
}


void TrajectoryVisualizer::AddGoal(
    visualization_msgs::MarkerArray& msg,
    const Eigen::Vector2d& goal)
{
  Marker marker;
  marker = GenerateMarker(goal, visualization_msgs::Marker::CYLINDER, 0.03);
  marker.ns = "goal";
  marker.scale.z = 0.1;
  marker.color.a = 0.5;
  msg.markers.push_back(marker);
}


visualization_msgs::Marker
TrajectoryVisualizer::GenerateMarker(Eigen::Vector2d pos, int32_t type, double size) const
{
  visualization_msgs::Marker marker;
  marker.pose.position.x = pos.x();
  marker.pose.position.y = pos.y();
  marker.pose.position.z = 0.0;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ::ros::Time();
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
//  marker.lifetime = ::ros::Duration(0.01);
  marker.scale.x = marker.scale.y = marker.scale.z = size;
  marker.color.a = 1.0;

  return marker;
}

void
TrajectoryVisualizer::AddLineStrip(visualization_msgs::MarkerArray& msg,
                           double center_x, double depth_x,
                           const std::string& rviz_namespace) const
{
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = frame_id_;
  line_strip.header.stamp = ::ros::Time::now();
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.ns = rviz_namespace;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.x = line_strip.pose.orientation.y = line_strip.pose.orientation.z = 0.0;
  line_strip.pose.orientation.w = 1.0;
  line_strip.color.b = 1.0;
  line_strip.color.a = 0.2;
  line_strip.scale.x = depth_x;
  geometry_msgs::Point p1, p2;
  p1.x = center_x;
  p1.y = -0.5;
  p1.z = 0;

  p2 = p1;
  p2.y = -p1.y;

  line_strip.points.push_back(p1);
  line_strip.points.push_back(p2);
  msg.markers.push_back(line_strip);
}




void
TrajectoryVisualizer::AddCogTrajectory(visualization_msgs::MarkerArray& msg,
                            const VecSpline& splines,
                            const std::vector<xpp::hyq::Foothold>& H_footholds,
                            const std::string& rviz_namespace,
                            double alpha)
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (double t(0.0); t < SplineContainer::GetTotalTime(splines); t+= 0.02)
  {

    xpp::utils::Point2d cog_state = SplineContainer::GetCOGxy(t, splines);
    int id = SplineContainer::GetSplineID(t, splines);



    visualization_msgs::Marker marker;
    marker = GenerateMarker(cog_state.p.segment<2>(0),
                            visualization_msgs::Marker::SPHERE,
                            0.005);
    marker.id = i++;
    marker.ns = rviz_namespace;


    bool four_legg_support = splines.at(id).IsFourLegSupport();
    if ( four_legg_support ) {
      marker.color.r = marker.color.g = marker.color.b = 0.1;
      if (splines.at(id).GetType() == xpp::zmp::Initial4lsSpline)
        marker.color.g = marker.color.b = 1.0;
    } else {
      int step = splines.at(id).GetCurrStep();
      xpp::hyq::LegID swing_leg = H_footholds.at(step).leg;
      marker.color = GetLegColor(swing_leg);
    }

    marker.color.a = alpha;
    msg.markers.push_back(marker);
  }
}


void
TrajectoryVisualizer::AddZmpTrajectory(visualization_msgs::MarkerArray& msg,
                            const VecSpline& splines,
                            const std::vector<xpp::hyq::Foothold>& H_footholds,
                            const std::string& rviz_namespace,
                            double alpha)
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (double t(0.0); t < SplineContainer::GetTotalTime(splines); t+= 0.1)
  {
    xpp::utils::Point2d cog_state = SplineContainer::GetCOGxy(t, splines);

    Eigen::Vector2d zmp = xpp::zmp::ZeroMomentPoint::CalcZmp(cog_state.Make3D(), walking_height_);

    int id = SplineContainer::GetSplineID(t, splines);

    visualization_msgs::Marker marker;
    marker = GenerateMarker(zmp,
                            visualization_msgs::Marker::CUBE,
                            0.005);

    marker.ns = rviz_namespace;
    marker.id = i++;

    if ( splines.at(id).IsFourLegSupport() ) {
      marker.color.r = marker.color.g = marker.color.g = 0.1;
      if (splines.at(id).GetType() == xpp::zmp::Initial4lsSpline)
        marker.color.g = marker.color.b = 1.0;
    } else {
      int step = splines.at(id).GetCurrStep();
      xpp::hyq::LegID swing_leg = H_footholds.at(step).leg;
      marker.ns = "leg " + std::to_string(swing_leg);
      marker.color = GetLegColor(swing_leg);
    }

    marker.color.a = alpha;
    msg.markers.push_back(marker);
  }
}


void TrajectoryVisualizer::AddFootholds(
    visualization_msgs::MarkerArray& msg,
    const std::vector<xpp::hyq::Foothold>& H_footholds,
    const std::string& rviz_namespace,
    int32_t type,
    double alpha)
{

  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (uint j=0; j<H_footholds.size(); ++j) {

    ::geometry_msgs::Point f_curr;
    f_curr.x = H_footholds.at(j).p.x();
    f_curr.y = H_footholds.at(j).p.y();
    f_curr.z = H_footholds.at(j).p.z();

//    ::geometry_msgs::Point f_next1;
//    f_next1.x = H_footholds.at(j+1).p.x();
//    f_next1.y = H_footholds.at(j+1).p.y();
//    f_next1.z = H_footholds.at(j+1).p.z();
//
//    ::geometry_msgs::Point f_next2;
//    f_next2.x = H_footholds.at(j+1).p.x();
//    f_next2.y = H_footholds.at(j+1).p.y();
//    f_next2.z = H_footholds.at(j+1).p.z();
//
//    ::geometry_msgs::Point f_next3;
//    f_next3.x = H_footholds.at(j+1).p.x();
//    f_next3.y = H_footholds.at(j+1).p.y();
//    f_next3.z = H_footholds.at(j+1).p.z();


    // publish to rviz
    visualization_msgs::Marker marker_msg;
    marker_msg.type = type;
    marker_msg.action = visualization_msgs::Marker::ADD;

    marker_msg.pose.position = f_curr;
//    marker_msg.points.push_back(f_next1);
//
//    marker_msg.points.push_back(f_curr);
//    marker_msg.points.push_back(f_next2);
//
//    marker_msg.points.push_back(f_curr);
//    marker_msg.points.push_back(f_next3);


    marker_msg.header.frame_id = frame_id_;  // name of the tf message that defines the location of the body with respect to the april tag
    marker_msg.header.stamp = ::ros::Time();
    marker_msg.ns = rviz_namespace;
    marker_msg.id = i++;
//    marker_msg.lifetime = ros::Duration(10);
    marker_msg.scale.x = 0.05;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;


    marker_msg.color = GetLegColor(H_footholds.at(j).leg);
    marker_msg.color.a = alpha;

    msg.markers.push_back(marker_msg);
  }
}


std_msgs::ColorRGBA TrajectoryVisualizer::GetLegColor(xpp::hyq::LegID leg) const
{
  // define a few colors
  std_msgs::ColorRGBA red, green, blue, yellow, white;
  red.a = green.a = blue.a = yellow.a = white.a = 1.0;

  red.r = 1.0;
  green.g = 1.0;
  blue.b = 1.0;
  yellow.r = yellow.g = 1.0;
  white.b = white.g = white.r = 1.0;


  std_msgs::ColorRGBA color_leg;
  switch (leg) {
    case xpp::hyq::LF:
      color_leg = red;
      break;
    case xpp::hyq::RF:
      color_leg = green;
      break;
    case xpp::hyq::LH:
      color_leg = blue;
      break;
    case xpp::hyq::RH:
      color_leg = yellow;
      break;
    default:
      break;
  }

  return color_leg;
}


} /* namespace ros */
} /* namespace xpp */
