/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/transport_hints.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Int32.h>

//#include <kindr/rotations/Rotation.hpp>

#include <xpp/optimization_parameters.h>
#include <xpp/state.h>
#include <xpp/ros/ros_conversions.h>
#include <xpp/ros/topic_names.h>
#include <xpp/height_map.h>

namespace xpp {
namespace ros {


NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(xpp_msgs::user_command, 1,
                                &NlpOptimizerNode::UserCommandCallback, this);
  ROS_INFO_STREAM("Subscribed to " << user_command_sub_.getTopic());

  current_state_sub_ = n.subscribe(xpp_msgs::robot_state_current,
                                   1, // take only the most recent information
                                   &NlpOptimizerNode::CurrentStateCallback, this,
                                   ::ros::TransportHints().tcpNoDelay());
  ROS_INFO_STREAM("Subscribed to " << current_state_sub_.getTopic());

  cart_trajectory_pub_  = n.advertise<xpp_msgs::RobotStateCartesianTrajectory>
                                          (xpp_msgs::robot_trajectory_desired, 1);

  opt_parameters_pub_  = n.advertise<xpp_msgs::OptParameters>
                                    (xpp_msgs::opt_parameters, 1);

  dt_          = RosConversions::GetDoubleFromServer("/xpp/trajectory_dt");
  rosbag_name_ = RosConversions::GetStringFromServer("/xpp/rosbag_name") + ".bag";
}

void
NlpOptimizerNode::CurrentStateCallback (const xpp_msgs::RobotStateCartesian& msg)
{
  auto curr_state = RosConversions::RosToXpp(msg);
  motion_optimizer_.initial_ee_W_ = curr_state.GetEEPos();

  motion_optimizer_.inital_base_     = State3dEuler(); // zero
  motion_optimizer_.inital_base_.lin = curr_state.base_.lin;

//  kindr::RotationQuaternionD quat(curr_state.base_.ang.q);
//  kindr::EulerAnglesZyxD euler(quat);
//  euler.setUnique(); // to express euler angles close to 0,0,0, not 180,180,180 (although same orientation)

  motion_optimizer_.inital_base_.ang.p_ = GetEulerZYXAngles(curr_state.base_.ang.q);
  ROS_INFO_STREAM("Received Current State. Setting only positions, zeroing velocity and accelerations");
}

void
NlpOptimizerNode::UserCommandCallback(const xpp_msgs::UserCommand& msg)
{
  user_command_msg_ = msg;

  motion_optimizer_.terrain_ = opt::HeightMap::MakeTerrain(static_cast<opt::HeightMap::ID>(msg.terrain_id));
  // assume flat ground and make sure feet stay at initial height
  double avg_height=0.0;
  for ( auto pos : motion_optimizer_.initial_ee_W_.ToImpl())
    avg_height += pos.z()/motion_optimizer_.initial_ee_W_.GetCount();
  motion_optimizer_.terrain_->SetGroundHeight(avg_height);
  ROS_INFO_STREAM("Ground height assumed at " << avg_height);


  motion_optimizer_.model_->SetInitialGait(msg.gait_id);
  motion_optimizer_.params_->SetTotalDuration(msg.total_duration);

  motion_optimizer_.nlp_solver_ = msg.use_solver_snopt ? MotionOptimizerFacade::Snopt : MotionOptimizerFacade::Ipopt;
  motion_optimizer_.SetFinalState(RosConversions::RosToXpp(msg.goal_lin), RosConversions::RosToXpp(msg.goal_ang));

  ROS_INFO_STREAM("publishing optimization parameters to " << opt_parameters_pub_.getTopic());
  opt_parameters_pub_.publish(BuildOptParametersMsg());

  if (msg.publish_traj) {
    ROS_INFO_STREAM("publishing optimized trajectory to " << cart_trajectory_pub_.getTopic());
    cart_trajectory_pub_.publish(BuildTrajectoryMsg());
  }

  if (msg.optimize) {
    OptimizeMotion();
    SaveOptimizationAsRosbag();
  }

  if (msg.replay_trajectory || msg.optimize) {
    // play back the rosbag hacky like this, as I can't find appropriate C++ API.
    system(("rosbag play --quiet " + rosbag_name_).c_str());
  }
}

void
NlpOptimizerNode::OptimizeMotion ()
{
  try {
    motion_optimizer_.SolveProblem();
  } catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Optimization failed, not sending. " << e.what());
  }
}

xpp_msgs::RobotStateCartesianTrajectory
NlpOptimizerNode::BuildTrajectoryMsg () const
{
  auto cart_traj = motion_optimizer_.GetTrajectory(dt_);
  return RosConversions::XppToRos(cart_traj);
}

xpp_msgs::OptParameters
NlpOptimizerNode::BuildOptParametersMsg() const
{
  xpp_msgs::OptParameters params_msg;
  auto max_dev_xyz = motion_optimizer_.model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = RosConversions::XppToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = motion_optimizer_.model_->GetNominalStanceInBase();
  auto ee_names = motion_optimizer_.model_->GetEndeffectorNames();
  params_msg.ee_names = ee_names;
  for (auto ee : nominal_B.ToImpl()) {
    params_msg.nominal_ee_pos.push_back(RosConversions::XppToRos<geometry_msgs::Point>(ee));
  }

  params_msg.base_mass = motion_optimizer_.model_->GetMass();

  return params_msg;
}

void
NlpOptimizerNode::SaveOptimizationAsRosbag () const
{
  rosbag::Bag bag;
  bag.open(rosbag_name_, rosbag::bagmode::Write);
  ::ros::Time t0(0.001);

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::opt_parameters, t0, BuildOptParametersMsg());
  bag.write(xpp_msgs::user_command+"_saved", t0, user_command_msg_);

  // save the trajectory of each iteration
  auto trajectories = motion_optimizer_.GetIntermediateSolutions(dt_);
  int n_iterations = trajectories.size();
  for (int i=0; i<n_iterations; ++i)
    SaveTrajectoryInRosbag(bag, trajectories.at(i), xpp_msgs::nlp_iterations_name + std::to_string(i));

  // save number of iterations the optimizer took
  std_msgs::Int32 m;
  m.data = n_iterations;
  bag.write(xpp_msgs::nlp_iterations_count, t0, m);

  // save the final trajectory
  auto final_trajectory = motion_optimizer_.GetTrajectory(dt_);
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  // optional: save the entire trajectory as one message
  bag.write(xpp_msgs::robot_trajectory_desired, t0, BuildTrajectoryMsg());

  bag.close();
}

void
NlpOptimizerNode::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                          const std::vector<RobotStateCartesian>& traj,
                                          const std::string& topic) const
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ +1e-6); // to avoid t=0.0

    auto state_msg = ros::RosConversions::XppToRos(state);
    bag.write(topic, timestamp, state_msg);
  }
}


} /* namespace ros */
} /* namespace xpp */
