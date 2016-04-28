/*
 * ros_helpers.h
 *
 *  Created on: Apr 8, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_

#include <ros/ros.h>

#include <xpp_opt/Spline.h>
#include <xpp_opt/Footholds2d.h>
#include <xpp_opt/StateLin3d.h>
#include <xpp_opt/Foothold.h>

#include <xpp/utils/geometric_structs.h>
#include <xpp/hyq/foothold.h>
#include <xpp/hyq/leg_data_map.h>
#include <xpp/zmp/spline_container.h>

namespace xpp {
namespace ros {

/**
 * Ros specific functions that only depend on utils folder, ros messages,
 * ros services.
 */
struct RosHelpers {

typedef Eigen::Vector3d Vector3d;
typedef xpp::utils::Point3d State;
typedef xpp::hyq::Foothold Foothold;
typedef xpp::zmp::SplineContainer::VecSpline VecSpline;
typedef xpp_opt::Spline SplineMsg;

static double GetDoubleFromServer(const std::string& ros_param_name) {
  double val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetDoubleFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}


static std::vector<SplineMsg>
XppToRos(const VecSpline& opt_splines)
{
  using namespace xpp::zmp;

  int n_splines = opt_splines.size();
  std::vector<SplineMsg> msgs(n_splines);

  for (uint i=0; i<opt_splines.size(); ++i)
  {
    const double* ax_coeff = opt_splines.at(i).spline_coeff_[xpp::utils::X];
    std::copy(ax_coeff, ax_coeff+xpp::zmp::kCoeffCount, msgs.at(i).coeff_x.begin());

    const double* ay_coeff = opt_splines.at(i).spline_coeff_[xpp::utils::Y];
    std::copy(ay_coeff, ay_coeff+xpp::zmp::kCoeffCount, msgs.at(i).coeff_y.begin());

    msgs.at(i).duration = opt_splines.at(i).duration_;
    msgs.at(i).four_leg_support = opt_splines.at(i).four_leg_supp_;
    msgs.at(i).id = opt_splines.at(i).id_;
    msgs.at(i).step = opt_splines.at(i).step_;
  }

  return msgs;
}


static VecSpline
RosToXpp(const std::vector<SplineMsg>& msgs)
{
  using namespace xpp::zmp;

  uint n_splines = msgs.size();
  VecSpline xpp(n_splines);

  for (uint i=0; i<n_splines; ++i)
  {
    const double* ax_coeff = msgs.at(i).coeff_x.begin();
    std::copy(ax_coeff, ax_coeff+xpp::zmp::kCoeffCount, xpp.at(i).spline_coeff_[xpp::utils::X]);

    const double* ay_coeff = msgs.at(i).coeff_y.begin();
    std::copy(ay_coeff, ay_coeff+xpp::zmp::kCoeffCount, xpp.at(i).spline_coeff_[xpp::utils::Y]);

    xpp.at(i).duration_      = msgs.at(i).duration;
    xpp.at(i).four_leg_supp_ = msgs.at(i).four_leg_support;
    xpp.at(i).id_            = msgs.at(i).id;
    xpp.at(i).step_          = msgs.at(i).step;
  }

  return xpp;
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
  point.p.z() = ros.pos.z;

  point.v.x() = ros.vel.x;
  point.v.y() = ros.vel.y;
  point.v.z() = ros.vel.z;

  point.a.x() = ros.acc.x;
  point.a.y() = ros.acc.y;
  point.a.z() = ros.acc.z;

  return point;
}


static xpp_opt::StateLin3d
XppToRos(const State& xpp)
{
  xpp_opt::StateLin3d ros;
  ros.pos.x = xpp.p.x();
  ros.pos.y = xpp.p.y();
  ros.pos.z = xpp.p.z();

  ros.vel.x = xpp.v.x();
  ros.vel.y = xpp.v.y();
  ros.vel.z = xpp.v.z();

  ros.acc.x = xpp.a.x();
  ros.acc.y = xpp.a.y();
  ros.acc.z = xpp.a.z();

  return ros;
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


static std::vector<xpp_opt::Foothold>
XppToRos(const std::vector<xpp::hyq::Foothold>& xpp)
{
  int n_footholds = xpp.size();
  std::vector<xpp_opt::Foothold> ros_vec(n_footholds);

  for (int i=0; i<n_footholds; ++i) {
    ros_vec.at(i) = XppToRos(xpp.at(i));
  }

  return ros_vec;
}


static std::vector<xpp::hyq::Foothold>
RosToXpp(const std::vector<xpp_opt::Foothold>& ros)
{
  std::vector<xpp::hyq::Foothold> xpp_vec(ros.size());

  for (uint i=0; i<ros.size(); ++i) {
    xpp_vec.at(i) = RosToXpp(ros.at(i));
  }

  return xpp_vec;
}


}; // RosHelpers


}
}


#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_ */
