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

#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>


namespace towr {


/**
 * @brief Base class to interface TOWR with a ROS GUI and RVIZ.
 *
 * This is very convenient to change goal states or terrains on the fly and
 * test how your formulation holds up. A sample application implementing this
 * interface is TowrRosApp.
 */
class TowrRosInterface {
public:
  using XppVec         = std::vector<xpp::RobotStateCartesian>;
  using TowrCommandMsg = towr_ros::TowrCommand;
  using Vector3d       = Eigen::Vector3d;

protected:
  TowrRosInterface ();
  virtual ~TowrRosInterface () = default;

  /**
   * @brief Sets the base state and end-effector position.
   */
  virtual void SetTowrInitialState() = 0;

  /**
   * @brief Formulates the actual TOWR problem to be solved
   * @param msg User message to adjust the parameters dynamically.
   *
   * When formulating your own application, here you can set your specific
   * set of constraints and variables.
   */
  virtual Parameters GetTowrParameters(int n_ee, const TowrCommandMsg& msg) const = 0;

  /**
   * @brief Sets the parameters of the nonlinear programming solver IPOPT.
   * @param msg User message that can be used to change the parameters.
   */
  virtual void SetIpoptParameters(const TowrCommandMsg& msg) = 0;

  NlpFormulation formulation_;         ///< the default formulation, can be adapted
  ifopt::IpoptSolver::Ptr solver_; ///< NLP solver, could also use SNOPT.

private:
  SplineHolder solution; ///< the solution splines linked to the opt-variables.
  ifopt::Problem nlp_;   ///< the actual nonlinear program to be solved.
  double visualization_dt_; ///< duration between two rviz visualization states.

  ::ros::Subscriber user_command_sub_;
  ::ros::Publisher initial_state_pub_;
  ::ros::Publisher robot_parameters_pub_;

  void UserCommandCallback(const TowrCommandMsg& msg);
  XppVec GetTrajectory() const;
  virtual BaseState GetGoalState(const TowrCommandMsg& msg) const;
  void PublishInitialState();
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
