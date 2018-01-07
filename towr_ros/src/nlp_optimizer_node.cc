/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <towr_ros/nlp_optimizer_node.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <ctime>

#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/transport_hints.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Int32.h>

#include <xpp_states/state.h>
#include <xpp_states/convert.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr_ros/param_server.h>
#include <towr_ros/topic_names.h>

#include <towr/constraints/height_map.h>

namespace towr {

using namespace xpp;


NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(xpp_msgs::user_command, 1,
                                  &NlpOptimizerNode::UserCommandCallback, this);
  ROS_INFO_STREAM("Subscribed to " << user_command_sub_.getTopic());

  current_state_sub_ = n.subscribe(xpp_msgs::robot_state_current,
                                   1, // take only the most recent information
                                   &NlpOptimizerNode::CurrentStateCallback, this);
  ROS_INFO_STREAM("Subscribed to " << current_state_sub_.getTopic());

  cart_trajectory_pub_  = n.advertise<xpp_msgs::RobotStateCartesianTrajectory>
                                          (xpp_msgs::robot_trajectory_desired, 1);

  robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                    (xpp_msgs::robot_parameters, 1);

  dt_            = ParamServer::GetDouble("/towr/trajectory_dt");
  rosbag_folder_ = ParamServer::GetString("/towr/rosbag_name");



  // hardcode initial state
  RobotStateCartesian init(4);
  init.base_.lin.p_.z() = 0.46;

  init.ee_motion_.at(0).p_ <<  0.34,  0.19, 0.0; // LF
  init.ee_motion_.at(1).p_ <<  0.34, -0.19, 0.0; // RF
  init.ee_motion_.at(2).p_ << -0.34,  0.19, 0.0; // LH
  init.ee_motion_.at(3).p_ << -0.34, -0.19, 0.0; // RH

  motion_optimizer_.SetInitialState(init);
}

void
NlpOptimizerNode::CurrentStateCallback (const xpp_msgs::RobotStateCartesian& msg)
{
  auto curr_state = Convert::ToXpp(msg);

//  // some logging of the real robot state sent from SL
//  if (save_current_state_)
//    curr_states_log_.push_back(curr_state);
//
//  // end of current batch
//  if (curr_state.t_global_ > user_command_msg_.total_duration-10*dt_) {
//    // get current date and time
//    std::string subfolder = "/published/";
//    time_t _tm = time(NULL );
//    struct tm * curtime = localtime ( &_tm );
//    std::string name = rosbag_folder_ + subfolder + asctime(curtime) + "_real.bag";
//
//    rosbag::Bag bag;
//    bag.open(name, rosbag::bagmode::Write);
//    SaveTrajectoryInRosbag(bag, curr_states_log_, xpp_msgs::robot_state_current);
//    bag.close();
//    save_current_state_ = false; // reached the end of trajectory
//  }

  motion_optimizer_.SetInitialState(curr_state);
//  if (first_callback_) {
//    first_callback_ = false;
//  }


//  std::cout << "current footholds:\n";
//  for (auto ee : msg.ee_motion) {
//    std::cout << ee.pos.z << "\t";
//  }
//  std::cout << std::endl;
}

void
NlpOptimizerNode::UserCommandCallback(const UserCommand& msg)
{
  auto terrain = HeightMap::MakeTerrain(static_cast<TerrainID>(msg.terrain_id));

  State3dEuler final_base;
  final_base.lin = Convert::ToXpp(msg.goal_lin);
  final_base.ang = Convert::ToXpp(msg.goal_ang);


  RobotModel model;
  model.MakeAnymalModel();
  model.gait_generator_->SetCombo(static_cast<GaitGenerator::GaitCombos>(msg.gait_id));
  //  model_.MakeMonopedModel();
  //  model_.MakeBipedModel();
  //  model_.MakeHyqModel();

  ROS_INFO_STREAM("publishing optimization parameters to " << robot_parameters_pub_.getTopic());
  xpp_msgs::RobotParameters robot_params_msg = BuildRobotParametersMsg(model);
  robot_parameters_pub_.publish(robot_params_msg);


  motion_optimizer_.SetParameters(final_base, msg.total_duration, model, terrain);




  std::string bag_file = rosbag_folder_+ bag_name_;
  if (msg.optimize) {
    OptimizeMotion();
    SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, terrain, false);
  }

  if (msg.replay_trajectory || msg.optimize) {
    // play back the rosbag hacky like this, as I can't find appropriate C++ API.
    system(("rosbag play --topics " + xpp_msgs::robot_state_desired + " "
                                    + xpp_msgs::terrain_info
                                    + " --quiet " + bag_file).c_str());
  }


  if (msg.publish_traj) {
    ROS_INFO_STREAM("publishing optimized trajectory to " << cart_trajectory_pub_.getTopic());
    auto traj_msg = BuildTrajectoryMsg();
    cart_trajectory_pub_.publish(traj_msg);

//    // this is where next motion will start from
//    auto final_state = motion_optimizer_.GetTrajectory(dt_).back();
//    motion_optimizer_.SetInitialState(final_state);
//
//    curr_states_log_.clear();
//    save_current_state_ = true;

    // get current date and time
    std::string subfolder = "/published/";
    time_t _tm =time(NULL );
    struct tm * curtime = localtime ( &_tm );
    std::string name = rosbag_folder_ + subfolder + asctime(curtime) + ".bag";
    SaveOptimizationAsRosbag(name, robot_params_msg, msg, terrain, false);
  }
}

void
NlpOptimizerNode::OptimizeMotion ()
{
  try {
    motion_optimizer_.SolveNLP();
  } catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Optimization failed, not sending. " << e.what());
  }
}

xpp_msgs::RobotStateCartesianTrajectory
NlpOptimizerNode::BuildTrajectoryMsg () const
{
  auto cart_traj = motion_optimizer_.GetTrajectory(dt_);
  return Convert::ToRos(cart_traj);
}

xpp_msgs::RobotParameters
NlpOptimizerNode::BuildRobotParametersMsg(const RobotModel& model) const
{
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  auto ee_names = model.gait_generator_->GetEndeffectorNames();
  params_msg.ee_names = ee_names;
  for (auto ee : nominal_B.ToImpl()) {
    params_msg.nominal_ee_pos.push_back(Convert::ToRos<geometry_msgs::Point>(ee));
  }

  params_msg.base_mass = model.dynamic_model_->GetMass();

  return params_msg;
}

void
NlpOptimizerNode::SaveOptimizationAsRosbag (const std::string& bag_name,
                                            const xpp_msgs::RobotParameters& robot_params,
                                            const UserCommand user_command_msg,
                                            const HeightMap::Ptr& terrain,
                                            bool include_iterations) const
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(0.001);

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(xpp_msgs::user_command+"_saved", t0, user_command_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = motion_optimizer_.GetIntermediateSolutions(dt_);
    int n_iterations = trajectories.size();
    for (int i=0; i<n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i), terrain, xpp_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(xpp_msgs::nlp_iterations_count, t0, m);
  }


  // save the final trajectory
  auto final_trajectory = motion_optimizer_.GetTrajectory(dt_);
  SaveTrajectoryInRosbag(bag, final_trajectory, terrain, xpp_msgs::robot_state_desired);

  // optional: save the entire trajectory as one message
  bag.write(xpp_msgs::robot_trajectory_desired, t0, BuildTrajectoryMsg());

  bag.close();
}

void
NlpOptimizerNode::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                          const std::vector<RobotStateCartesian>& traj,
                                          const HeightMap::Ptr& terrain,
                                          const std::string& topic) const
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ +1e-6); // to avoid t=0.0

    xpp_msgs::RobotStateCartesian msg;
    msg = Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n = terrain->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = terrain->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);



//    Vector3d f = motion_optimizer_.terrain_->GetHeight(state.)

//    bag.write(topic, timestamp, msg);
  }
}

} /* namespace xpp */
