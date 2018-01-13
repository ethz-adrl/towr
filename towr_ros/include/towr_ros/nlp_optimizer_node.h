/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the ROS interface to motion optimizer.
 */

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

#include <towr_ros/UserCommand.h>    // receive from user

#include <towr/towr.h>

namespace xpp {

class NlpOptimizerNode {
public:
  using UserCommand         = towr_ros::UserCommand;
  using RobotStateCartesian = xpp::RobotStateCartesian;
  using RobotStateVec       = std::vector<RobotStateCartesian>;
  using HeightMap           = towr::HeightMap;
  using RobotModel          = towr::RobotModel;

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

  towr::TOWR towr_;
  double dt_; ///< discretization of output trajectory (1/TaskServoHz)

  double total_time_;

  bool first_callback_ = true;

  std::string rosbag_folder_; ///< folder to save bags
  std::string bag_name_ = "/optimal_traj.bag";
  std::vector<RobotStateCartesian> curr_states_log_;
  bool save_current_state_ = false;

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


  towr::BaseState ToBaseState(const xpp::State3d& base) const;
  towr::BaseState ToBaseState(const xpp::State3dEuler& base) const;

  StateLinXd ToXpp(const towr::StateLinXd& towr) const;
//  xpp::StateAng3d ToXpp(const towr::StateAng3d& towr) const;

  StateAng3d GetState (const towr::StateLinXd& euler) const;


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

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
