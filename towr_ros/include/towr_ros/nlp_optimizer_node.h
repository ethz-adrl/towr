/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include <towr_ros/TowrCommand.h>    // receive from user
#include <towr_ros/gait_generator.h>

#include <towr/towr.h>

namespace towr {

class NlpOptimizerNode {
public:
  using UserCommand         = towr_ros::TowrCommand;
  using RobotStateCartesian = xpp::RobotStateCartesian;
  using RobotStateVec       = std::vector<RobotStateCartesian>;
  using HeightMap           = towr::HeightMap;
  using RobotModel          = towr::RobotModel;
  using Vector3d            = Eigen::Vector3d;

  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () = default;

private:
  void OptimizeMotion();

  void CurrentStateCallback(const xpp_msgs::RobotStateCartesian&);

  void UserCommandCallback(const UserCommand&);

  ::ros::Subscriber user_command_sub_;
  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher cart_trajectory_pub_;
  ::ros::Publisher robot_parameters_pub_;

  TOWR towr_;






  double dt_; ///< discretization of output trajectory (1/TaskServoHz)

  double total_time_;

  bool first_callback_ = true;

  std::string rosbag_folder_; ///< folder to save bags
  std::string bag_name_ = "/optimal_traj.bag";
  std::vector<RobotStateCartesian> curr_states_log_;
  bool save_current_state_ = false;

  towr::GaitGenerator::Ptr gait_generator_;
  towr::RobotModel model_;

  xpp_msgs::RobotStateCartesianTrajectory BuildTrajectoryMsg() const;

  void SaveOptimizationAsRosbag(const std::string& bag_name,
                                const xpp_msgs::RobotParameters& robot_params,
                                const UserCommand user_command_msg,
                                const HeightMap::Ptr& terrain,
                                bool include_iterations=false);
  void SaveTrajectoryInRosbag (rosbag::Bag&, const std::vector<RobotStateCartesian>& traj,
                               const HeightMap::Ptr& terrain, const std::string& topic) const;
  xpp_msgs::RobotParameters BuildRobotParametersMsg(const RobotModel& model) const;


  RobotStateVec GetTrajectory() const;
  std::vector<RobotStateVec>GetIntermediateSolutions();


  towr::BaseState ToTOWR(const xpp::State3d& base) const;
  towr::BaseState ToTOWR(const xpp::State3dEuler& base) const;

  xpp::StateLinXd ToXpp(const towr::State& towr) const;



  Eigen::Vector3d GetUnique(const Eigen::Vector3d& zyx_non_unique) const;

//  void SetTerrainHeightFromAvgFootholdHeight(
//      HeightMap::Ptr& terrain) const
//  {
//    double avg_height=0.0;
//    for ( auto pos : new_ee_pos_)
//      avg_height += pos.z()/new_ee_pos_.size();
//    terrain->SetGroundHeight(avg_height);
//  }
};

} /* namespace towr */

#endif /* XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
