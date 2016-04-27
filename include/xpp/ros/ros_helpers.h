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

  for (uint i=0; i<opt_splines.size(); ++i) {

    // FIXME copy these directly .data() or smth
    msgs.at(i).coeff_x.at(A) = opt_splines.at(i).spline_coeff_[xpp::utils::X][A];
    msgs.at(i).coeff_x.at(B) = opt_splines.at(i).spline_coeff_[xpp::utils::X][B];
    msgs.at(i).coeff_x.at(C) = opt_splines.at(i).spline_coeff_[xpp::utils::X][C];
    msgs.at(i).coeff_x.at(D) = opt_splines.at(i).spline_coeff_[xpp::utils::X][D];
    msgs.at(i).coeff_x.at(E) = opt_splines.at(i).spline_coeff_[xpp::utils::X][E];
    msgs.at(i).coeff_x.at(F) = opt_splines.at(i).spline_coeff_[xpp::utils::X][F];

    msgs.at(i).coeff_y.at(A) = opt_splines.at(i).spline_coeff_[xpp::utils::Y][A];
    msgs.at(i).coeff_y.at(B) = opt_splines.at(i).spline_coeff_[xpp::utils::Y][B];
    msgs.at(i).coeff_y.at(C) = opt_splines.at(i).spline_coeff_[xpp::utils::Y][C];
    msgs.at(i).coeff_y.at(D) = opt_splines.at(i).spline_coeff_[xpp::utils::Y][D];
    msgs.at(i).coeff_y.at(E) = opt_splines.at(i).spline_coeff_[xpp::utils::Y][E];
    msgs.at(i).coeff_y.at(F) = opt_splines.at(i).spline_coeff_[xpp::utils::Y][F];

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

  for (uint i=0; i<n_splines; ++i) {

    // FIXME copy these directly .data() or smth
    xpp.at(i).spline_coeff_[xpp::utils::X][A] = msgs.at(i).coeff_x.at(A);
    xpp.at(i).spline_coeff_[xpp::utils::X][B] = msgs.at(i).coeff_x.at(B);
    xpp.at(i).spline_coeff_[xpp::utils::X][C] = msgs.at(i).coeff_x.at(C);
    xpp.at(i).spline_coeff_[xpp::utils::X][D] = msgs.at(i).coeff_x.at(D);
    xpp.at(i).spline_coeff_[xpp::utils::X][E] = msgs.at(i).coeff_x.at(E);
    xpp.at(i).spline_coeff_[xpp::utils::X][F] = msgs.at(i).coeff_x.at(F);

    xpp.at(i).spline_coeff_[xpp::utils::Y][A] = msgs.at(i).coeff_y.at(A);
    xpp.at(i).spline_coeff_[xpp::utils::Y][B] = msgs.at(i).coeff_y.at(B);
    xpp.at(i).spline_coeff_[xpp::utils::Y][C] = msgs.at(i).coeff_y.at(C);
    xpp.at(i).spline_coeff_[xpp::utils::Y][D] = msgs.at(i).coeff_y.at(D);
    xpp.at(i).spline_coeff_[xpp::utils::Y][E] = msgs.at(i).coeff_y.at(E);
    xpp.at(i).spline_coeff_[xpp::utils::Y][F] = msgs.at(i).coeff_y.at(F);

    xpp.at(i).duration_      = msgs.at(i).duration;
    xpp.at(i).four_leg_supp_ = msgs.at(i).four_leg_support;
    xpp.at(i).id_            = msgs.at(i).id;
    xpp.at(i).step_          = msgs.at(i).step;
  }

  return xpp;
}


//static xpp_opt::SplineCoefficients
//XppToRos(const Eigen::VectorXd& opt_coefficients)
//{
//  xpp_opt::SplineCoefficients msg;
//  for (int i=0; i<opt_coefficients.rows(); ++i)
//    msg.data.push_back(opt_coefficients[i]);
//
//  return msg;
//}


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
  std::vector<xpp_opt::Foothold> ros_vec(xpp.size());

  for (uint i=0; i<xpp.size(); ++i) {
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


static boost::array<xpp_opt::Foothold,4>
XppToRos(const xpp::hyq::LegDataMap<Foothold>& xpp)
{
  boost::array<xpp_opt::Foothold,4> ros;
  for (uint i=0; i<ros.size(); ++i) {
    int leg = xpp[i].leg;
    ros.at(leg) = XppToRos(xpp[i]);
  }

  return ros;
}


}; // RosHelpers


}
}


#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_ */
