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

#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace ros {

/**
 * Ros specific functions that only depend on utils folder, ros messages,
 * ros services.
 */
struct RosHelpers {

typedef xpp::utils::Point2d State;

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
  xpp_opt::Footholds2d msg;
  for (uint i=0; i<opt_footholds.size(); ++i) {
    geometry_msgs::Point p;
    p.x = opt_footholds.at(i).x();
    p.y = opt_footholds.at(i).y();
    msg.data.push_back(p);
  }

  return msg;
}


static State
RosToXpp(const xpp_opt::StateLin3d& msg)
{
  State point;
  point.p.x() = msg.pos.x;
  point.p.y() = msg.pos.y;

  point.v.x() = msg.vel.x;
  point.v.y() = msg.vel.y;

  point.a.x() = msg.acc.x;
  point.a.y() = msg.acc.y;

  return point;
}

}; // RosHelpers


}
}


#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_ */
