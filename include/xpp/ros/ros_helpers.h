/*
 * ros_helpers.h
 *
 *  Created on: Apr 8, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_

#include <ros/ros.h>

#include <xpp_opt/SplineCoefficients.h>
#include <xpp_opt/Footholds2d.h>
#include <xpp_opt/StateLin3d.h>
#include <xpp_opt/Foothold.h>

#include <xpp/utils/geometric_structs.h>
#include <xpp/hyq/foothold.h>
#include <xpp/hyq/leg_data_map.h>

namespace xpp {
namespace ros {

/**
 * Ros specific functions that only depend on utils folder, ros messages,
 * ros services.
 */
struct RosHelpers {

typedef Eigen::Vector3d Vector3d;
typedef xpp::utils::Point2d State;
typedef xpp::hyq::Foothold Foothold;

static double GetDoubleFromServer(const std::string& ros_param_name) {
  double val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetDoubleFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}


static xpp_opt::SplineCoefficients
XppToRos(const Eigen::VectorXd& opt_coefficients)
{
  xpp_opt::SplineCoefficients msg;
  for (int i=0; i<opt_coefficients.rows(); ++i)
    msg.data.push_back(opt_coefficients[i]);

  return msg;
}


static xpp_opt::Footholds2d
XppToRos(const xpp::utils::StdVecEigen2d& opt_footholds)
{
  xpp_opt::Footholds2d ros;
  for (uint i=0; i<opt_footholds.size(); ++i) {
    geometry_msgs::Point p;
    p.x = opt_footholds.at(i).x();
    p.y = opt_footholds.at(i).y();
    ros.data.push_back(p);
  }

  return ros;
}


static State
RosToXpp(const xpp_opt::StateLin3d& ros)
{
  State point;
  point.p.x() = ros.pos.x;
  point.p.y() = ros.pos.y;

  point.v.x() = ros.vel.x;
  point.v.y() = ros.vel.y;

  point.a.x() = ros.acc.x;
  point.a.y() = ros.acc.y;

  return point;
}


static Vector3d
RosToXpp(const geometry_msgs::Point& ros)
{
  Vector3d vec;
  vec << ros.x, ros.y, ros.z;
  return vec;
}


static Foothold
RosToXpp(const xpp_opt::Foothold& ros)
{
  Foothold f;
  f.leg = static_cast<xpp::hyq::LegID>(ros.leg);
  f.p = RosToXpp(ros.p);
  return f;
}


static xpp_opt::Foothold
XppToRos(const xpp::hyq::Foothold& xpp)
{
  xpp_opt::Foothold ros;
  ros.p.x = xpp.p.x();
  ros.p.y = xpp.p.y();
  ros.p.z = xpp.p.z();
  ros.leg = xpp.leg;

  return ros;
}


static xpp::hyq::LegDataMap<Foothold>
RosToXpp(const boost::array<xpp_opt::Foothold,4>& ros)
{
  xpp::hyq::LegDataMap<Foothold> xpp;
  for (uint i=0; i<ros.size(); ++i) {
    int leg = ros.at(i).leg;
    xpp[leg] = RosToXpp(ros.at(i));
  }

  return xpp;
}


}; // RosHelpers


}
}


#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_ */
