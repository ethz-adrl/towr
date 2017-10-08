/**
 @file    ros_helpers.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    April 8, 2016
 @brief   Defines conversions between ROS and xpp data types
 */

#ifndef INCLUDE_XPP_MSGS_ROS_HELPERS_H_
#define INCLUDE_XPP_MSGS_ROS_HELPERS_H_

#include <ros/ros.h>

#include <xpp_msgs/StateLin3d.h>
#include <xpp_msgs/State6d.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>

#include <xpp/state.h>
#include <xpp/robot_state_cartesian.h>

namespace xpp {
namespace ros {

/**
 * Ros specific functions that only depend on utils folder, ros messages,
 * ros services.
 */
struct RosConversions {

static double GetDoubleFromServer(const std::string& ros_param_name) {
  double val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetDoubleFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}

static bool GetBoolFromServer(const std::string& ros_param_name) {
  bool val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetBoolFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}

static std::string GetStringFromServer(const std::string& ros_param_name) {
  std::string val;
  if(!::ros::param::get(ros_param_name,val)) {
    ROS_ERROR_STREAM("GetStringFromServer: Couldn't read parameter: " << ros_param_name);
    ROS_INFO_STREAM("\nIngore?");
    std::cin.get(); // wait for user input
//    throw ::ros::Exception("GetStringFromServer: Couldn't read parameter: " + ros_param_name);
  }
  return val;
}


static StateLin3d
RosToXpp(const xpp_msgs::StateLin3d& ros)
{
  StateLin3d point;
  point.p_.x() = ros.pos.x;
  point.p_.y() = ros.pos.y;
  point.p_.z() = ros.pos.z;

  point.v_.x() = ros.vel.x;
  point.v_.y() = ros.vel.y;
  point.v_.z() = ros.vel.z;

  point.a_.x() = ros.acc.x;
  point.a_.y() = ros.acc.y;
  point.a_.z() = ros.acc.z;

  return point;
}

static xpp_msgs::StateLin3d
XppToRos(const StateLin3d& xpp)
{
  xpp_msgs::StateLin3d ros;
  ros.pos.x = xpp.p_.x();
  ros.pos.y = xpp.p_.y();
  ros.pos.z = xpp.p_.z();

  ros.vel.x = xpp.v_.x();
  ros.vel.y = xpp.v_.y();
  ros.vel.z = xpp.v_.z();

  ros.acc.x = xpp.a_.x();
  ros.acc.y = xpp.a_.y();
  ros.acc.z = xpp.a_.z();

  return ros;
}

template<typename T>
static Vector3d
RosToXpp(const T& ros)
{
  Vector3d vec;
  vec << ros.x, ros.y, ros.z;
  return vec;
}

template<typename T>
static T
XppToRos(const Vector3d& xpp)
{
  T ros;
  ros.x = xpp.x();
  ros.y = xpp.y();
  ros.z = xpp.z();

  return ros;
}

static Eigen::Quaterniond
RosToXpp(const geometry_msgs::Quaternion ros)
{
  Eigen::Quaterniond xpp;
  xpp.w() = ros.w;
  xpp.x() = ros.x;
  xpp.y() = ros.y;
  xpp.z() = ros.z;

  return xpp;
}

static geometry_msgs::Quaternion
XppToRos(const Eigen::Quaterniond xpp)
{
  geometry_msgs::Quaternion ros;
  ros.w = xpp.w();
  ros.x = xpp.x();
  ros.y = xpp.y();
  ros.z = xpp.z();

  return ros;
}

static xpp_msgs::State6d
XppToRos(const State3d& xpp)
{
  xpp_msgs::State6d msg;

  msg.pose.position = XppToRos<geometry_msgs::Point>(xpp.lin.p_);
  msg.twist.linear  = XppToRos<geometry_msgs::Vector3>(xpp.lin.v_);
  msg.accel.linear  = XppToRos<geometry_msgs::Vector3>(xpp.lin.a_);

  msg.pose.orientation = XppToRos(xpp.ang.q);
  msg.twist.angular    = XppToRos<geometry_msgs::Vector3>(xpp.ang.v);
  msg.accel.angular    = XppToRos<geometry_msgs::Vector3>(xpp.ang.a);

  return msg;
}

static State3d
RosToXpp(const xpp_msgs::State6d& ros)
{
  State3d xpp;

  xpp.lin.p_ = RosToXpp(ros.pose.position);
  xpp.lin.v_ = RosToXpp(ros.twist.linear);
  xpp.lin.a_ = RosToXpp(ros.accel.linear);

  xpp.ang.q = RosToXpp(ros.pose.orientation);
  xpp.ang.v = RosToXpp(ros.twist.angular);
  xpp.ang.a = RosToXpp(ros.accel.angular);

  return xpp;
}

static xpp_msgs::RobotStateCartesian
XppToRos(const RobotStateCartesian& xpp)
{
  xpp_msgs::RobotStateCartesian ros;

  ros.base            = XppToRos(xpp.base_);
  ros.time_from_start = ::ros::Duration(xpp.t_global_);

  for (auto ee : xpp.ee_contact_.GetEEsOrdered()) {
    ros.ee_motion. push_back(XppToRos(xpp.ee_motion_.At(ee)));
    ros.ee_contact.push_back(xpp.ee_contact_.At(ee));
    ros.ee_forces. push_back(XppToRos<geometry_msgs::Vector3>(xpp.ee_forces_.At(ee)));
  }

  return ros;
}

static RobotStateCartesian
RosToXpp(const xpp_msgs::RobotStateCartesian& ros)
{

  int n_ee = ros.ee_motion.size();
  RobotStateCartesian xpp(n_ee);

  xpp.base_     = RosToXpp(ros.base);
  xpp.t_global_ = ros.time_from_start.toSec();

  for (auto ee : xpp.ee_contact_.GetEEsOrdered()) {
    xpp.ee_motion_.At(ee)  = RosToXpp(ros.ee_motion.at(ee));
    xpp.ee_contact_.At(ee) = ros.ee_contact.at(ee);
    xpp.ee_forces_.At(ee)  = RosToXpp(ros.ee_forces.at(ee));
  }

  return xpp;
}

static xpp_msgs::RobotStateCartesianTrajectory
XppToRos(const std::vector<RobotStateCartesian>& xpp)
{
  xpp_msgs::RobotStateCartesianTrajectory msg;

  for (const auto state : xpp) {
    auto state_msg = XppToRos(state);
    msg.points.push_back(state_msg);
  }

  return msg;
}

static std::vector<RobotStateCartesian>
RosToXpp(const xpp_msgs::RobotStateCartesianTrajectory& ros)
{
  std::vector<RobotStateCartesian> xpp_vec;

  for (const auto ros_state : ros.points) {
    auto xpp = RosToXpp(ros_state);
    xpp_vec.push_back(xpp);
  }

  return xpp_vec;
}



//using RobotStateCommonMsg = xpp_msgs::RobotStateCommon;
//
//static RobotStateCommon
//RosToXpp(const RobotStateCommonMsg& ros)
//{
//  int n_ee            = ros.ee_in_contact.size();
//  RobotStateCommon xpp(n_ee);
//
//  xpp.base_          = RosToXpp(ros.base);
//  xpp.t_global_      = ros.t_global;
//
//  RobotStateCommon::ContactState contact_state(n_ee);
//  for (int ee=0; ee<n_ee; ++ee)
//    xpp.is_contact_.At(static_cast<EEID>(ee)) = ros.ee_in_contact.at(ee);
//
//  return xpp;
//}
//
//static RobotStateCommonMsg
//XppToRos(const RobotStateCommon& xpp)
//{
//  RobotStateCommonMsg ros;
//
//  ros.base          = XppToRos(xpp.base_);
//  ros.t_global      = xpp.t_global_;
//
//  for (auto ee : xpp.is_contact_.GetEEsOrdered()) {
//    ros.ee_in_contact.push_back(xpp.is_contact_.At(ee));
//  }
//
//  return ros;
//}


//using RobotStateCartesianMsg     = xpp_msgs::RobotStateCartesian;
//
//static RobotStateCartesianMsg
//XppToRos(const RobotStateCartesian& xpp)
//{
//  RobotStateCartesianMsg ros;
//
//  ros.common = XppToRos(xpp.GetCommon());
//
//  for (auto ee : xpp.GetEEState().ToImpl())
//    ros.feet.push_back(XppToRos(ee));
//
//  for (auto ee : xpp.GetEEForces().ToImpl())
//    ros.ee_forces.push_back(XppToRos<geometry_msgs::Vector3>(ee));
//
//  return ros;
//}
//
//static RobotStateCartesian
//RosToXpp(const RobotStateCartesianMsg& ros)
//{
//  int n_ee = ros.feet.size();
//  RobotStateCartesian xpp(n_ee);
//
//  xpp.SetCommon(RosToXpp(ros.common));
//
//  RobotStateCartesian::FeetArray feet(n_ee);
//  int i=0;
//  for (auto state : ros.feet)
//    feet.At(static_cast<EEID>(i++)) = RosToXpp(state);
//  xpp.SetEEStateInWorld(feet);
//
//  i=0;
//  RobotStateCartesian::EEForces ee_forces(n_ee);
//  for (auto forces : ros.ee_forces)
//    ee_forces.At(static_cast<EEID>(i++)) = RosToXpp(forces);
//  xpp.SetEEForcesInWorld(ee_forces);
//
//  return xpp;
//}















//using RobotStateCartesianTrajMsg = xpp_msgs::RobotStateCartesianTrajectory;
//
//static RobotStateCartesianTrajMsg
//XppToRosCart(const std::vector<RobotStateCartesian>& xpp_traj)
//{
//  RobotStateCartesianTrajMsg msg;
//  for (auto& state : xpp_traj)
//    msg.states.push_back(XppToRos(state));
//
//  return msg;
//}
//
//static std::vector<RobotStateCartesian>
//RosToXppCart(const RobotStateCartesianTrajMsg& msg)
//{
//  std::vector<RobotStateCartesian> xpp;
//
//  for (const auto& state : msg.states)
//    xpp.push_back(RosToXpp(state));
//
//  return xpp;
//}



//using ContactMsg       = xpp_msgs::Contact;
//using ContactVectorMsg = xpp_msgs::ContactVector;
//
//static Contact
//RosToXpp(const ContactMsg& ros)
//{
//  Contact xpp;
//
//  xpp.id  = ros.id;
//  xpp.ee  = static_cast<EEID>(ros.ee);
//  xpp.p   = RosToXpp(ros.p);
//  return xpp;
//}
//
//static ContactMsg
//XppToRos(const Contact& xpp)
//{
//  ContactMsg ros;
//  ros.id = xpp.id;
//  ros.ee = xpp.ee;
//  ros.p = XppToRos<geometry_msgs::Point>(xpp.p);
//
//  return ros;
//}
//
//static ContactVectorMsg
//XppToRos(const std::vector<Contact>& xpp)
//{
//  ContactVectorMsg ros;
//
//  for (auto x : xpp)
//    ros.contacts.push_back(XppToRos(x));
//
//  return ros;
//}
//
//static std::vector<Contact>
//RosToXpp(const ContactVectorMsg& ros)
//{
//  std::vector<Contact> xpp;
//
//  for (auto r : ros.contacts)
//    xpp.push_back(RosToXpp(r));
//
//  return xpp;
//}

//using VecComPoly = std::vector<xpp::utils::ComPolynomial>;
//using Polynomial = xpp::utils::Polynomial;
//using SplineMsg  = xpp_msgs::Spline;

//static std::vector<SplineMsg>
//XppToRos(const VecComPoly& opt_splines)
//{
//  int n_splines = opt_splines.size();
//  std::vector<SplineMsg> msgs(n_splines);
//
//  for (uint i=0; i<opt_splines.size(); ++i)
//  {
//
//    for (auto coeff : Polynomial::AllSplineCoeff) {
//      msgs.at(i).coeff_x[coeff] = opt_splines.at(i).GetCoefficient(xpp::utils::X,coeff);
//      msgs.at(i).coeff_y[coeff] = opt_splines.at(i).GetCoefficient(xpp::utils::Y,coeff);
//    }
//
//    msgs.at(i).duration = opt_splines.at(i).GetDuration();
//    msgs.at(i).id       = opt_splines.at(i).GetId();
//  }
//
//  return msgs;
//}
//
//static VecComPoly
//RosToXpp(const std::vector<SplineMsg>& msgs)
//{
//  uint n_splines = msgs.size();
//  VecComPoly xpp(n_splines);
//
//  for (uint i=0; i<n_splines; ++i)
//  {
//    for (auto coeff : Polynomial::AllSplineCoeff) {
//      xpp.at(i).SetCoefficients(xpp::utils::X, coeff, msgs.at(i).coeff_x[coeff]);
//      xpp.at(i).SetCoefficients(xpp::utils::Y, coeff, msgs.at(i).coeff_y[coeff]);
//    }
//    xpp.at(i).SetDuration(msgs.at(i).duration);
//    xpp.at(i).SetId(msgs.at(i).id);
//  }
//  return xpp;
//}


}; // RosHelpers

} // namespace ros
} // namespace xpp

#endif /* INCLUDE_XPP_MSGS_ROS_HELPERS_H_ */
