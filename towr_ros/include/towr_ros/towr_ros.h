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

#ifndef TOWR_INCLUDE_TOWR_ROS_TOWR_ROS_H_
#define TOWR_INCLUDE_TOWR_ROS_TOWR_ROS_H_

#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <xpp_states/robot_state_cartesian.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotParameters.h>
#include <towr_ros/TowrCommand.h>

#include <towr/towr.h>
#include <ifopt/ipopt_solver.h>


namespace towr {

class TowrRos {
public:
  using XppVec         = std::vector<xpp::RobotStateCartesian>;
  using TowrCommandMsg = towr_ros::TowrCommand;
  using Vector3d       = Eigen::Vector3d;

  TowrRos ();
  virtual ~TowrRos () = default;

private:
  void UserCommandCallback(const TowrCommandMsg& msg);

  XppVec GetTrajectory() const;

  // publishing to rviz with ROS bag
  ::ros::Subscriber user_command_sub_;
  ::ros::Publisher initial_state_pub_;
  ::ros::Publisher robot_parameters_pub_;

  void SetInitialFromNominal(const std::vector<Vector3d>& nomial_stance_B);
  void PublishInitial();
  BaseState initial_base_;
  std::vector<Vector3d> initial_ee_pos_;

  HeightMap::Ptr terrain_;
  TOWR towr_;
  ifopt::IpoptSolver::Ptr solver_;
  double visualization_dt_; ///< discretization of output trajectory (1/TaskServoHz)

private:
  std::vector<XppVec>GetIntermediateSolutions();
  xpp_msgs::RobotParameters BuildRobotParametersMsg(const RobotModel& model) const;

  void SaveOptimizationAsRosbag(const std::string& bag_name,
                                const xpp_msgs::RobotParameters& robot_params,
                                const TowrCommandMsg user_command_msg,
                                bool include_iterations=false);
  void SaveTrajectoryInRosbag (rosbag::Bag&,
                               const std::vector<xpp::RobotStateCartesian>& traj,
                               const std::string& topic) const;
};

} /* namespace towr */

#endif /* XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
