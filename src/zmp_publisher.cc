/*
 * zmp_publisher.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_publisher.h>

namespace xpp {
namespace zmp {

ZmpPublisher::ZmpPublisher (const xpp::zmp::ContinuousSplineContainer& trajectory)
{
  trajectory_ = trajectory;
  zmp_msg_.markers.clear();

  ros::NodeHandle n;
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
  AddTrajectory(msg, trajectory_, rviz_namespace, alpha);


  zmp_msg_.markers.insert(zmp_msg_.markers.end(),
                          msg.markers.begin(),msg.markers.end());
}


void
ZmpPublisher::AddTrajectory(visualization_msgs::MarkerArray& msg,
                   xpp::zmp::SplineContainer zmp_splines,
                   const std::string& rviz_namespace,
                   double alpha)
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (double t(0.0); t < zmp_splines.GetTotalTime(); t+= 0.03)
  {
    xpp::utils::Point2d cog_state;
    zmp_splines.GetCOGxy(t, cog_state);


    visualization_msgs::Marker marker;
    marker.id = i++;
    marker.pose.position.x = cog_state.p.x();
    marker.pose.position.y = cog_state.p.y();
    marker.pose.position.z = 0.0;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time();
    marker.ns = rviz_namespace;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
//    marker.lifetime = ros::Duration(10);
    marker.scale.x = marker.scale.y = marker.scale.z = 0.005;

    std_msgs::ColorRGBA red;
    red.a = alpha;
    red.r = 1.0;
    marker.color = red;

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
    marker_msg.header.stamp = ros::Time();
    marker_msg.ns = rviz_namespace;
    marker_msg.id = i++;
//    marker_msg.lifetime = ros::Duration(10);
    marker_msg.scale.x = 0.05;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;


    std_msgs::ColorRGBA red, green, blue, yellow, white;
    red.a = green.a = blue.a = yellow.a = white.a = alpha;

    red.r = 1.0;
    green.g = 1.0;
    blue.b = 1.0;
    yellow.r = yellow.g = 1.0;
    white.b = white.g = white.r = 1.0;


    switch (H_footholds.at(j).leg) {
      case xpp::hyq::LF:
        marker_msg.color = red;
        break;
      case xpp::hyq::RF:
        marker_msg.color = green;
        break;
      case xpp::hyq::LH:
        marker_msg.color = blue;
        break;
      case xpp::hyq::RH:
        marker_msg.color = yellow;
        break;
      default:
        break;

    }

    msg.markers.push_back(marker_msg);
  }
}



} /* namespace zmp */
} /* namespace xpp */
