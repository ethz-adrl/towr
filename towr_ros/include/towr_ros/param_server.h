
#ifndef XPP_OPT_ROS_PARAM_SERVER_H_
#define XPP_OPT_ROS_PARAM_SERVER_H_

#include <ros/ros.h>

namespace xpp {

/**
 * @brief Functions to retrieve values from the ROS parameter server.
 *
 * These are usually specified and loaed through a launch file as:
 * <param name="my_string"  value="some_string" type="string"/>
 * <param name="my_double"  value="0.0"         type="double"/>
 * <param name="my_bool"    value="true"        type="bool"/>
 */
class ParamServer {
public:
  ParamServer ();
  virtual ~ParamServer ();

  /**
   * @brief Returns the boolean value of the parameter ros_param_name.
   */
  static bool GetBool(const std::string& ros_param_name) {
    bool val;
    if(!ros::param::get(ros_param_name,val))
      throw ros::Exception("GetBoolFromServer: Couldn't read parameter: " + ros_param_name);
    return val;
  }

  /**
   * @brief Returns the double value of the parameter ros_param_name.
   */
  static double GetDouble(const std::string& ros_param_name) {
    double val;
    if(!ros::param::get(ros_param_name,val))
      throw ros::Exception("GetDoubleFromServer: Couldn't read parameter: " + ros_param_name);
    return val;
  }

  /**
   * @brief Returns the string of the parameter ros_param_name.
   */
  static std::string GetString(const std::string& ros_param_name) {
    std::string val;
    if(!ros::param::get(ros_param_name,val))
      throw ::ros::Exception("GetStringFromServer: Couldn't read parameter: " + ros_param_name);
    return val;
  }
};

} /* namespace xpp */

#endif /* XPP_OPT_ROS_PARAM_SERVER_H_ */
