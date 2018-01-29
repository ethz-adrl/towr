/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef XPP_OPT_ROS_PARAM_SERVER_H_
#define XPP_OPT_ROS_PARAM_SERVER_H_

#include <ros/ros.h>

namespace towr {

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

} /* namespace towr */

#endif /* XPP_OPT_ROS_PARAM_SERVER_H_ */
