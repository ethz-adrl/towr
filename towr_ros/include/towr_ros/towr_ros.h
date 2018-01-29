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

#ifndef XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_
#define XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_

#include <string>

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <rosbag/bag.h>

#include <xpp_states/robot_state_cartesian.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/RobotParameters.h>

#include <towr_ros/TowrCommand.h>
#include <towr_ros/gait_generator.h>

#include <towr/towr.h>

namespace towr {

class TowrRos {
public:
  using RobotStateCartesian = xpp::RobotStateCartesian;
  using RobotStateVec       = std::vector<RobotStateCartesian>;
  using TowrCommand         = towr_ros::TowrCommand;
  using Vector3d            = Eigen::Vector3d;

  TowrRos ();
  virtual ~TowrRos () = default;

private:
  void OptimizeMotion();

  void CurrentStateCallback(const xpp_msgs::RobotStateCartesian&);

  void UserCommandCallback(const TowrCommand&);
  void SetTowrParameters(const TowrCommand& msg);

  RobotStateVec GetTrajectory() const;
  std::vector<RobotStateVec>GetIntermediateSolutions();


  ::ros::Subscriber user_command_sub_;
  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher cart_trajectory_pub_;
  ::ros::Publisher robot_parameters_pub_;


  TOWR towr_;

  GaitGenerator::Ptr gait_;
  RobotModel model_;
  HeightMap::Ptr terrain_;

  double output_dt_; ///< discretization of output trajectory (1/TaskServoHz)
  std::string rosbag_folder_; ///< folder to save bags




  xpp_msgs::RobotStateCartesianTrajectory BuildTrajectoryMsg() const;

  xpp_msgs::RobotParameters BuildRobotParametersMsg(const RobotModel& model) const;

  void SaveOptimizationAsRosbag(const std::string& bag_name,
                                const xpp_msgs::RobotParameters& robot_params,
                                const TowrCommand user_command_msg,
                                bool include_iterations=false);
  void SaveTrajectoryInRosbag (rosbag::Bag&, const std::vector<RobotStateCartesian>& traj,
                               const std::string& topic) const;



  xpp::StateLinXd ToXpp(const towr::State& towr) const;



  Eigen::Vector3d GetUnique(const Eigen::Vector3d& zyx_non_unique) const;
};

} /* namespace towr */

#endif /* XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
