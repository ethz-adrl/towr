/*
 * ros_helpers.h
 *
 *  Created on: Apr 8, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_

#include <ros/ros.h>

#include <xpp_opt/OptimizedVariables.h>
#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace ros {

/**
 * Ros specific functions that only depend on utils folder, ros messages,
 * ros services.
 */
struct RosHelpers {

  static double GetDoubleFromServer(const std::string& ros_param_name) {
    double val;
    if(!::ros::param::get(ros_param_name,val))
      throw ::ros::Exception("GetDoubleFromServer: Couldn't read parameter: " + ros_param_name);
    return val;
  }

  static xpp_opt::OptimizedVariables
  XppToRos(const Eigen::VectorXd& opt_coefficients,
           const xpp::utils::StdVecEigen2d& opt_footholds)
  {
    xpp_opt::OptimizedVariables x;
    //FIXME don't use for loop, copy data directly
    for (int i=0; i<opt_coefficients.rows(); ++i)
      x.spline_coeff.push_back(opt_coefficients[i]);
    for (int i=0; i<opt_footholds.size(); ++i) {
      geometry_msgs::Point p;
      p.x = opt_footholds.at(i).x();
      p.y = opt_footholds.at(i).y();
      x.footholds.push_back(p);
    }

    return x;
  }

}; // RosHelpers


}
}


#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ROS_HELPERS_H_ */
