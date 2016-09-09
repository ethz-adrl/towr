/*
 * ros_helpers.h
 *
 *  Created on: Apr 8, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_

#include <ros/ros.h>

#include <xpp_opt/PhaseInfo.h>
#include <xpp_opt/Contact.h>
#include <xpp_opt/Spline.h>
#include <xpp_opt/StateLin3d.h>
#include <xpp_opt/Foothold.h>

#include <xpp/utils/geometric_structs.h>
#include <xpp/zmp/phase_info.h>
#include <xpp/hyq/foothold.h>
#include <xpp/hyq/leg_data_map.h>
#include <xpp/zmp/com_spline.h>

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
typedef xpp::zmp::ComSpline::VecPolynomials VecSpline;
typedef xpp_opt::Spline SplineMsg;
typedef xpp::hyq::LegID LegID;

using ContactMsg   = xpp_opt::Contact;
using ContactXpp   = xpp::zmp::Contact;
using PhaseInfoMsg = xpp_opt::PhaseInfo;
using PhaseInfoXpp = xpp::zmp::PhaseInfo;


static double GetDoubleFromServer(const std::string& ros_param_name) {
  double val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetDoubleFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}

static double GetBoolFromServer(const std::string& ros_param_name) {
  bool val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetBoolFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}

static ContactMsg
XppToRos(const ContactXpp& xpp)
{
  ContactMsg msg;
  msg.id = xpp.id;
  msg.ee = static_cast<int>(xpp.ee);

  return msg;
}

static ContactXpp
RosToXpp(const ContactMsg& msg)
{
  ContactXpp xpp;
  xpp.id = msg.id;
  xpp.ee = static_cast<xpp::zmp::EndeffectorID>(msg.ee);

  return xpp;
}

static PhaseInfoMsg
XppToRos(const PhaseInfoXpp& xpp)
{
  PhaseInfoMsg msg;
  msg.n_completed_steps = xpp.n_completed_steps_;
  for (auto c : xpp.free_contacts_)  msg.free_contacts.push_back(XppToRos(c));
  for (auto f : xpp.fixed_contacts_) msg.fixed_contacts.push_back(XppToRos(f));
  msg.id                = xpp.id_;
  msg.duration          = xpp.duration_;

  return msg;
}

static PhaseInfoXpp
RosToXpp(const PhaseInfoMsg& msg)
{
  PhaseInfoXpp xpp;
  xpp.n_completed_steps_ = msg.n_completed_steps;
  for (auto c : msg.free_contacts)  xpp.free_contacts_.push_back(RosToXpp(c));
  for (auto f : msg.fixed_contacts) xpp.fixed_contacts_.push_back(RosToXpp(f));
  xpp.id_                = msg.id;
  xpp.duration_          = msg.duration;

  return xpp;
}

static std::vector<PhaseInfoMsg>
XppToRos(const std::vector<PhaseInfoXpp>& xpp)
{
  std::vector<PhaseInfoMsg> msg;

  for (const auto& phase : xpp)
    msg.push_back(XppToRos(phase));

  return msg;
}

static std::vector<PhaseInfoXpp>
RosToXpp(const std::vector<PhaseInfoMsg>& msg)
{
  std::vector<PhaseInfoXpp> xpp;

  for (auto phase : msg)
    xpp.push_back(RosToXpp(phase));

  return xpp;
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
    msgs.at(i).id       = opt_splines.at(i).id_;
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

    xpp.at(i).duration_ = msgs.at(i).duration;
    xpp.at(i).id_       = msgs.at(i).id;
  }
  return xpp;
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

  assert(0 <= ros.leg && ros.leg < xpp::hyq::_LEGS_COUNT); //integer cannot be mapped to a LegID
  f.leg = static_cast<LegID>(ros.leg);
  f.p   = RosToXpp(ros.p);
  return f;
}

static xpp_opt::Foothold
XppToRos(const xpp::hyq::Foothold& xpp)
{
  xpp_opt::Foothold ros;
  ros.p.x = xpp.p.x();
  ros.p.y = xpp.p.y();
  ros.p.z = xpp.p.z();
  ros.leg = static_cast<int>(xpp.leg);

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
