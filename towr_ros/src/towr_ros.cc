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

#include <towr_ros/towr_ros.h>

#include <iostream>
#include <stdexcept>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr_ros/param_server.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/quadruped_gait_generator.h>
#include <towr_ros/height_map_examples.h>
#include <towr/variables/euler_converter.h> // smell move into spline_holder

#include <towr_ros/models/anymal_model.h>
#include <towr_ros/models/hyq_model.h>
#include <towr_ros/models/biped_model.h>
#include <towr_ros/models/monoped_model.h>

namespace towr {


TowrRos::TowrRos ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                  &TowrRos::UserCommandCallback, this);
  ROS_INFO_STREAM("Subscribed to " << user_command_sub_.getTopic());

  current_state_sub_ = n.subscribe(xpp_msgs::robot_state_current,
                                   1, // take only the most recent information
                                   &TowrRos::CurrentStateCallback, this);
  ROS_INFO_STREAM("Subscribed to " << current_state_sub_.getTopic());

  cart_trajectory_pub_  = n.advertise<xpp_msgs::RobotStateCartesianTrajectory>
                                          (xpp_msgs::robot_trajectory_desired, 1);

  robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                    (xpp_msgs::robot_parameters, 1);

  rosbag_folder_ = ParamServer::GetString("/towr/rosbag_folder");


  // hardcode initial state
  towr::BaseState b;
  b.lin.at(towr::kPos).z() = 0.58;

  std::vector<Eigen::Vector3d> ee_pos(4);

  ee_pos.at(0) <<  0.31,  0.29, 0.0; // LF
  ee_pos.at(1) <<  0.31, -0.29, 0.0; // RF
  ee_pos.at(2) << -0.31,  0.29, 0.0; // LH
  ee_pos.at(3) << -0.31, -0.29, 0.0; // RH

  towr_.SetInitialState(b, ee_pos);

  model_.dynamic_model_   = std::make_shared<towr::HyqDynamicModel>();
  model_.kinematic_model_ = std::make_shared<towr::HyqKinematicModel>();
  gait_                   = std::make_shared<towr::QuadrupedGaitGenerator>();
  output_dt_              = 0.0025; // 400 Hz control loop frequency on ANYmal

  terrain_ = std::make_shared<FlatGround>(0.0);
}

void
TowrRos::CurrentStateCallback (const xpp_msgs::RobotStateCartesian& msg)
{
  towr::BaseState base_initial;
  base_initial.lin.at(towr::kPos) = xpp::Convert::ToXpp(msg.base.pose.position);
  base_initial.lin.at(towr::kVel) = xpp::Convert::ToXpp(msg.base.twist.linear);

  Eigen::Quaterniond q = xpp::Convert::ToXpp(msg.base.pose.orientation);
  base_initial.ang.at(towr::kPos) = GetUnique(xpp::GetEulerZYXAngles(q));
  base_initial.ang.at(towr::kVel) = Vector3d::Zero(); // smell fill this angular_vel->euler rates

  std::vector<Vector3d> feet_initial;
  for (auto ee : msg.ee_motion)
    feet_initial.push_back(xpp::Convert::ToXpp(ee.pos));

  towr_.SetInitialState(base_initial, feet_initial);
}

void
TowrRos::UserCommandCallback(const TowrCommand& msg)
{
  SetTowrParameters(msg);


  ROS_INFO_STREAM("publishing robot parameters to " << robot_parameters_pub_.getTopic());
  xpp_msgs::RobotParameters robot_params_msg = BuildRobotParametersMsg(model_);
  robot_parameters_pub_.publish(robot_params_msg);


  std::string bag_file = rosbag_folder_+ "/optimal_traj.bag";
  if (msg.optimize) {
    OptimizeMotion();
    SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
  }

  if (msg.replay_trajectory || msg.optimize) {
    // play back the rosbag hacky like this, as I can't find appropriate C++ API.
    int success = system(("rosbag play --topics "
        + xpp_msgs::robot_state_desired + " "
        + xpp_msgs::terrain_info
        + " --quiet " + bag_file).c_str());
  }


  if (msg.publish_traj) {
    ROS_INFO_STREAM("publishing optimized trajectory to " << cart_trajectory_pub_.getTopic());
    auto traj_msg = BuildTrajectoryMsg();
    cart_trajectory_pub_.publish(traj_msg);

    // save published trajectory with current date and time
    std::string subfolder = "/published/";
    time_t _tm =time(NULL );
    struct tm * curtime = localtime ( &_tm );
    std::string name = rosbag_folder_ + subfolder + asctime(curtime) + ".bag";
    SaveOptimizationAsRosbag(name, robot_params_msg, msg, false);
  }
}

void
TowrRos::SetTowrParameters(const TowrCommand& msg)
{
  towr::BaseState goal;
  goal.lin.at(towr::kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
  goal.lin.at(towr::kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
  goal.ang.at(towr::kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
  goal.ang.at(towr::kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);


  towr::Parameters params;
  params.t_total_ = msg.total_duration;
  int n_ee = gait_->GetEndeffectorNames().size();
  auto gait = static_cast<towr::GaitGenerator::GaitCombos>(msg.gait_id);
  gait_->SetCombo(gait);
  for (int ee=0; ee<n_ee; ++ee) {
    params.ee_phase_durations_.push_back(gait_->GetPhaseDurations(msg.total_duration, ee));
    params.ee_in_contact_at_start_.push_back(gait_->IsInContactAtStart(ee));
  }


  double ground_height = 0.0; // must possibly be adapted for real robot
  auto terrain_id = static_cast<towr::TerrainID>(msg.terrain_id);
  terrain_ = towr::HeightMapFactory::MakeTerrain(terrain_id, ground_height);


  towr_.SetParameters(goal, params, model_, terrain_);
}

void
TowrRos::OptimizeMotion ()
{
  try {
    towr_.SolveNLP();
  } catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Optimization failed, not sending. " << e.what());
  }
}

std::vector<TowrRos::RobotStateVec>
TowrRos::GetIntermediateSolutions ()
{
  std::vector<RobotStateVec> trajectories;

  for (int iter=0; iter<towr_.GetIterationCount(); ++iter) {
    towr_.SetSolution(iter);
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

TowrRos::RobotStateVec
TowrRos::GetTrajectory () const
{
  towr::SplineHolder solution = towr_.GetSolution();

  RobotStateVec trajectory;
  double t=0.0;
  double T = solution.base_linear_->GetTotalTime();

  towr::EulerConverter base_angular(solution.base_angular_);

  while (t<=T+1e-5) {

    int n_ee = solution.ee_motion_.size();
    RobotStateCartesian state(n_ee);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (auto ee : state.ee_motion_.GetEEsOrdered()) {
      state.ee_contact_.at(ee) = solution.phase_durations_.at(ee)->IsContactPhase(t);
      state.ee_motion_.at(ee)  = ToXpp(solution.ee_motion_.at(ee)->GetPoint(t));
      state.ee_forces_ .at(ee)  = solution.ee_force_.at(ee)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += output_dt_;
  }

  return trajectory;
}

xpp::StateLinXd
TowrRos::ToXpp(const towr::State& towr) const
{
  xpp::StateLinXd xpp(towr.p().rows());

  xpp.p_ = towr.p();
  xpp.v_ = towr.v();
  xpp.a_ = towr.a();

  return xpp;
}

xpp_msgs::RobotStateCartesianTrajectory
TowrRos::BuildTrajectoryMsg () const
{
  auto cart_traj = GetTrajectory();
  return xpp::Convert::ToRos(cart_traj);
}

xpp_msgs::RobotParameters
TowrRos::BuildRobotParametersMsg(const RobotModel& model) const
{
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  auto ee_names = gait_->GetEndeffectorNames();
  params_msg.ee_names = ee_names;
  for (auto ee : nominal_B) {
    params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(ee));
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void
TowrRos::SaveOptimizationAsRosbag (const std::string& bag_name,
                                            const xpp_msgs::RobotParameters& robot_params,
                                            const TowrCommand user_command_msg,
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
      SaveTrajectoryInRosbag(bag, trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }


  // save the final trajectory
  auto final_trajectory = GetTrajectory();
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  // optional: save the entire trajectory as one message
  bag.write(xpp_msgs::robot_trajectory_desired, t0, BuildTrajectoryMsg());

  bag.close();
}

void
TowrRos::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                          const std::vector<RobotStateCartesian>& traj,
                                          const std::string& topic) const
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ +1e-6); // to avoid t=0.0

    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n = terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = terrain_->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
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
TowrRos::GetUnique (const Vector3d& zyx_non_unique) const
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

//  void SetTerrainHeightFromAvgFootholdHeight(
//      HeightMap::Ptr& terrain) const
//  {
//    double avg_height=0.0;
//    for ( auto pos : new_ee_pos_)
//      avg_height += pos.z()/new_ee_pos_.size();
//    terrain->SetGroundHeight(avg_height);
//  }

} /* namespace towr */


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "towr_ros");
  towr::TowrRos towr_ros;
  ros::spin();

  return 1;
}

