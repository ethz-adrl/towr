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
#include <xpp_states/endeffectors.h>
#include <xpp_states/convert.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr_ros/param_server.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/quadruped_gait_generator.h>

#include <towr_ros/height_map_examples.h>
#include <towr_ros/models/anymal_model.h>
#include <towr_ros/models/hyq_model.h>
#include <towr_ros/models/biped_model.h>
#include <towr_ros/models/monoped_model.h>
#include <towr/variables/euler_converter.h> // smell move into spline_holder

namespace towr {


NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
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
  towr::BaseState b;
  b.lin = b.ang = towr::Node(k3D);
  b.lin.at(towr::kPos).z() = 0.46;

  std::vector<Eigen::Vector3d> ee_pos(4);

  ee_pos.at(0) <<  0.34,  0.19, 0.0; // LF
  ee_pos.at(1) <<  0.34, -0.19, 0.0; // RF
  ee_pos.at(2) << -0.34,  0.19, 0.0; // LH
  ee_pos.at(3) << -0.34, -0.19, 0.0; // RH

  towr_.SetInitialState(b, ee_pos);

  model_.dynamic_model_   = std::make_shared<towr::AnymalDynamicModel>();
  model_.kinematic_model_ = std::make_shared<towr::AnymalKinematicModel>();
  gait_generator_         = std::make_shared<towr::QuadrupedGaitGenerator>();
}

void
NlpOptimizerNode::CurrentStateCallback (const xpp_msgs::RobotStateCartesian& msg)
{
  auto curr_state = xpp::Convert::ToXpp(msg);

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

//  SetInitialState(curr_state);


  towr::BaseState initial_base = ToTOWR(curr_state.base_);
  auto pos = curr_state.ee_motion_.Get(xpp::kPos).ToImpl();

  towr_.SetInitialState(initial_base, {pos.begin(), pos.end()} ); //





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
  double ground_height = 0.0; // must possibly be adapted for real robot
  auto terrain_id = static_cast<towr::TerrainID>(msg.terrain_id);
  auto terrain = towr::HeightMapFactory::MakeTerrain(terrain_id, ground_height);

  xpp::State3dEuler final_base;
  final_base.lin = xpp::Convert::ToXpp(msg.goal_lin);
  final_base.ang = xpp::Convert::ToXpp(msg.goal_ang);


  ROS_INFO_STREAM("publishing optimization parameters to " << robot_parameters_pub_.getTopic());
  xpp_msgs::RobotParameters robot_params_msg = BuildRobotParametersMsg(model_);
  robot_parameters_pub_.publish(robot_params_msg);

  total_time_ = msg.total_duration;
  towr::Parameters params;
  params.t_total_ = total_time_;


  int n_ee = gait_generator_->GetEndeffectorNames().size();
  gait_generator_->SetCombo(static_cast<towr::GaitGenerator::GaitCombos>(msg.gait_id));
  std::vector<bool> initial_contact;
  towr::GaitGenerator::FootDurations ee_durations;
  for (int ee=0; ee<n_ee; ++ee) {
    ee_durations.push_back(gait_generator_->GetPhaseDurations(total_time_, ee));
    initial_contact.push_back(gait_generator_->IsInContactAtStart(ee));
  }

  params.ee_phase_durations_ = ee_durations;
  params.ee_in_contact_at_start_ = initial_contact;






  towr_.SetParameters(ToTOWR(final_base), params, model_, terrain);




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
    towr_.SolveNLP(towr::TOWR::Solver::Ipopt);
  } catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Optimization failed, not sending. " << e.what());
  }
}

std::vector<NlpOptimizerNode::RobotStateVec>
NlpOptimizerNode::GetIntermediateSolutions ()
{
  std::vector<RobotStateVec> trajectories;

  for (int iter=0; iter<towr_.GetIterationCount(); ++iter) {
    towr_.SetSolution(iter); // this changes the values linked to the spline_holder
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

NlpOptimizerNode::RobotStateVec
NlpOptimizerNode::GetTrajectory () const
{
  towr::SplineHolder spline_holder_ = towr_.GetSolution();

  RobotStateVec trajectory;
  double t=0.0;
  double T = total_time_;//params_.GetTotalTime();

  towr::EulerConverter base_angular(spline_holder_.base_angular_);

  while (t<=T+1e-5) {

    int n_ee = spline_holder_.ee_motion_.size();
    RobotStateCartesian state(n_ee);

    // here of course the conversions will not work
    state.base_.lin = ToXpp(spline_holder_.base_linear_->GetPoint(t));

    state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (auto ee : state.ee_motion_.GetEEsOrdered()) {
      state.ee_contact_.at(ee) = spline_holder_.phase_durations_.at(ee)->IsContactPhase(t);
      state.ee_motion_.at(ee)  = ToXpp(spline_holder_.ee_motion_.at(ee)->GetPoint(t));
      state.ee_forces_.at(ee)  = spline_holder_.ee_force_.at(ee)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += dt_;
  }

  return trajectory;
}

xpp::StateLinXd
NlpOptimizerNode::ToXpp(const towr::State& towr) const
{
  xpp::StateLinXd xpp(3);

  xpp.p_ = towr.p();
  xpp.v_ = towr.v();
  xpp.a_ = towr.a();

  return xpp;
}

towr::BaseState
NlpOptimizerNode::ToTOWR(const xpp::State3d& base) const
{
  towr::BaseState b;

  b.lin.at(towr::kPos) = base.lin.p_;
  b.lin.at(towr::kVel) = base.lin.v_;

  b.ang.at(towr::kPos) = GetUnique(xpp::GetEulerZYXAngles(base.ang.q));
  b.ang.at(towr::kVel) = Vector3d::Zero(); // smell still fill this

  return b;
}

towr::BaseState
NlpOptimizerNode::ToTOWR(const xpp::State3dEuler& base) const
{
  towr::BaseState b;

  b.lin.at(towr::kPos) = base.lin.p_;
  b.lin.at(towr::kVel) = base.lin.v_;

  b.ang.at(towr::kPos) = base.ang.p_;
  b.ang.at(towr::kVel) = base.ang.v_;

  return b;
}

xpp_msgs::RobotStateCartesianTrajectory
NlpOptimizerNode::BuildTrajectoryMsg () const
{
  auto cart_traj = GetTrajectory();
  return xpp::Convert::ToRos(cart_traj);
}

xpp_msgs::RobotParameters
NlpOptimizerNode::BuildRobotParametersMsg(const RobotModel& model) const
{
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  auto ee_names = gait_generator_->GetEndeffectorNames();
  params_msg.ee_names = ee_names;
  for (auto ee : nominal_B) {
    params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(ee));
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void
NlpOptimizerNode::SaveOptimizationAsRosbag (const std::string& bag_name,
                                            const xpp_msgs::RobotParameters& robot_params,
                                            const UserCommand user_command_msg,
                                            const HeightMap::Ptr& terrain,
                                            bool include_iterations)
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(0.001);

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(towr_msgs::user_command+"_saved", t0, user_command_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = GetIntermediateSolutions();
    int n_iterations = trajectories.size();
    for (int i=0; i<n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i), terrain, towr_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }


  // save the final trajectory
  auto final_trajectory = GetTrajectory();
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
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n = terrain->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = terrain->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);



//    Vector3d f = motion_optimizer_.terrain_->GetHeight(state.)

//    bag.write(topic, timestamp, msg);
  }
}

/*! @brief Returns unique Euler angles in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
 *
 * Taken from https://github.com/ethz-asl/kindr
 *
 * Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
Eigen::Vector3d
NlpOptimizerNode::GetUnique (const Vector3d& zyx_non_unique) const
{
  Eigen::Vector3d zyx = zyx_non_unique;
  const double tol = 1e-3;

  // wrap angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
  if(zyx.y() < -M_PI/2 - tol)
  {
    if(zyx.x() < 0) {
      zyx.x() = zyx.x() + M_PI;
    } else {
      zyx.x() = zyx.x() - M_PI;
    }

    zyx.y() = -(zyx.y() + M_PI);

    if(zyx.z() < 0) {
      zyx.z() = zyx.z() + M_PI;
    } else {
      zyx.z() = zyx.z() - M_PI;
    }
  }
  else if(-M_PI/2 - tol <= zyx.y() && zyx.y() <= -M_PI/2 + tol)
  {
    zyx.x() -= zyx.z();
    zyx.z() = 0;
  }
  else if(-M_PI/2 + tol < zyx.y() && zyx.y() < M_PI/2 - tol)
  {
    // ok
  }
  else if(M_PI/2 - tol <= zyx.y() && zyx.y() <= M_PI/2 + tol)
  {
    // todo: M_PI/2 should not be in range, other formula?
    zyx.x() += zyx.z();
    zyx.z() = 0;
  }
  else // M_PI/2 + tol < zyx.y()
  {
    if(zyx.x() < 0) {
      zyx.x() = zyx.x() + M_PI;
    } else {
      zyx.x() = zyx.x() - M_PI;
    }

    zyx.y() = -(zyx.y() - M_PI);

    if(zyx.z() < 0) {
      zyx.z() = zyx.z() + M_PI;
    } else {
      zyx.z() = zyx.z() - M_PI;
    }
  }

  return zyx;
}

} /* namespace towr */
