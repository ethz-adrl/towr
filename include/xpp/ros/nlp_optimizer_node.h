/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the ROS interface to motion optimizer.
 */

#ifndef XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_
#define XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <rosbag/bag.h>

#include <xpp_msgs/OptParameters.h>
#include <xpp_msgs/RobotStateCartesian.h>  // receive from robot
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/UserCommand.h>          // receive from user

#include <xpp/opt/motion_optimizer_facade.h>
#include <xpp/robot_state_cartesian.h>

namespace xpp {
namespace ros {

class NlpOptimizerNode {
public:
  using StateMsg              = xpp_msgs::RobotStateCartesian;
  using UserCommandMsg        = xpp_msgs::UserCommand;
  using MotionOptimizerFacade = xpp::opt::MotionOptimizerFacade;
  using NlpSolver             = xpp::opt::NlpSolver;
  using TrajMsg               = xpp_msgs::RobotStateCartesianTrajectory;
  using RobotStateVec         = MotionOptimizerFacade::RobotStateVec;
  using ParamsMsg             = xpp_msgs::OptParameters;
  using NLPIterations         = MotionOptimizerFacade::NLPIterations;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:
//  void PublishTrajectory() const;
  void PublishOptParameters() const;

  void OptimizeMotion();
  void CurrentStateCallback(const StateMsg& msg);
  void UserCommandCallback(const UserCommandMsg& msg);

  ::ros::Subscriber user_command_sub_;
  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher cart_trajectory_pub_;
  ::ros::Publisher opt_parameters_pub_;

  MotionOptimizerFacade motion_optimizer_;
  double dt_; ///< discretization of output trajectory (1/TaskServoHz)
  std::string rosbag_name_; ///< folder to save bags

  NlpSolver solver_type_;

  void SetInitialState (const RobotStateCartesian& initial_state);

  void SaveOptimizationAsRosbag() const;
  void SaveTrajectoryInRosbag (rosbag::Bag&, const RobotStateVec& traj,
                               const std::string& topic) const;
  ParamsMsg BuildOptParameters() const;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
