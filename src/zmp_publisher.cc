/*
 * zmp_publisher.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: winklera
 */

#include <xpp/ros/zmp_publisher.h>

namespace xpp {
namespace ros {

ZmpPublisher::ZmpPublisher (const xpp::zmp::ContinuousSplineContainer& trajectory)
{
  trajectory_ = trajectory;
  zmp_msg_.markers.clear();

  ::ros::NodeHandle n;
  ros_publisher_ = n.advertise<visualization_msgs::MarkerArray>("zmp_trajectory", 1);
}

ZmpPublisher::~ZmpPublisher ()
{
  // TODO Auto-generated destructor stub
}


void ZmpPublisher::AddRvizMessage(
    const Eigen::VectorXd& opt_spline_coeff,
    const VecFoothold& opt_footholds,
    const std::string& rviz_namespace,
    double alpha)
{
  // update trajectory
  trajectory_.AddOptimizedCoefficients(opt_spline_coeff);


  visualization_msgs::MarkerArray msg;
  AddFootholds(msg, opt_footholds, rviz_namespace, visualization_msgs::Marker::CUBE, alpha);
  AddTrajectory(msg, trajectory_, opt_footholds, rviz_namespace, alpha);

  //FIXME move this somehwere else
  double strip_center = 0.3;
  double strip_depth  = 0.1;
  AddLineStrip(msg, strip_center, strip_depth);

  zmp_msg_.markers.insert(zmp_msg_.markers.end(),
                          msg.markers.begin(),msg.markers.end());
}

void
ZmpPublisher::AddLineStrip(visualization_msgs::MarkerArray& msg, double center_x, double depth_x) const
{
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = frame_id_;
  line_strip.header.stamp = ::ros::Time::now();
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.ns = "gap";
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
ZmpPublisher::AddTrajectory(visualization_msgs::MarkerArray& msg,
                   xpp::zmp::SplineContainer zmp_splines,
                   const std::vector<xpp::hyq::Foothold>& H_footholds,
                   const std::string& rviz_namespace,
                   double alpha)
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (double t(0.0); t < zmp_splines.GetTotalTime(); t+= 0.02)
  {

    xpp::utils::Point2d cog_state;
    zmp_splines.GetCOGxy(t, cog_state);

    visualization_msgs::Marker marker;
    marker.id = i++;
    marker.pose.position.x = cog_state.p.x();
    marker.pose.position.y = cog_state.p.y();
    marker.pose.position.z = 0.0;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ::ros::Time();
    marker.ns = rviz_namespace;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
//    marker.lifetime = ros::Duration(10);
    marker.scale.x = marker.scale.y = marker.scale.z = 0.005;


    bool four_legg_support = zmp_splines.GetFourLegSupport(t);
    if ( four_legg_support ) {
      marker.color.r = marker.color.g = marker.color.g = 0.1;
    } else {
      int step = zmp_splines.GetStep(t);
      xpp::hyq::LegID swing_leg = H_footholds.at(step).leg;
      marker.color = GetLegColor(swing_leg);
    }

    marker.color.a = alpha;
    msg.markers.push_back(marker);
  }
}


void ZmpPublisher::AddFootholds(
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


std_msgs::ColorRGBA ZmpPublisher::GetLegColor(xpp::hyq::LegID leg) const
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
