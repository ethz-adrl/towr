/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>

#include <xpp/ros/ros_visualizer.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>
#include <xpp_msgs/RobotStateTrajectory.h> // publish

#include <xpp/hyq/codegen/hyq_kinematics.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>

namespace xpp {
namespace ros {

using TrajectoryMsg = xpp_msgs::RobotStateTrajectory;
using RobotState = xpp::opt::RobotStateJoints;

static bool CheckIfInDirectoyWithIpoptConfigFile();

NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(xpp_msgs::goal_state_topic, 1,
                                &NlpOptimizerNode::UserCommandCallback, this);

  current_state_sub_ = n.subscribe(xpp_msgs::curr_robot_state,
                                    1, // take only the most recent information
                                    &NlpOptimizerNode::CurrentStateCallback, this,
                                    ::ros::TransportHints().tcpNoDelay());

  pub_ = n.advertise<PoseMsg>(xpp_msgs::curr_base_pose, 1);

  trajectory_pub_ = n.advertise<TrajectoryMsg>(xpp_msgs::robot_trajectory_joints, 1);

  dt_ = RosHelpers::GetDoubleFromServer("/xpp/trajectory_dt");

  motion_optimizer_.SetVisualizer(std::make_shared<RosVisualizer>());

  CheckIfInDirectoyWithIpoptConfigFile();

  ROS_INFO_STREAM("Initialization done, waiting for current state...");
}

void
NlpOptimizerNode::CurrentStateCallback (const CurrentInfoMsg& msg)
{
  auto curr_state = RosHelpers::RosToXpp(msg.state);
  auto fk = std::make_shared<hyq::codegen::HyQKinematics>();

  motion_optimizer_.BuildOptimizationStartState(curr_state.ConvertToCartesian(fk));
  ROS_INFO_STREAM("Received Current State");
  OptimizeMotion();
  PublishTrajectory(true);
}

void
NlpOptimizerNode::OptimizeMotion ()
{
  try {
    motion_optimizer_.OptimizeMotion(solver_type_);
//    PublishTrajectory ();
  } catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Optimization failed, not sending. " << e.what());
  }
}

void
NlpOptimizerNode::UserCommandCallback(const UserCommandMsg& msg)
{
  auto goal_prev = motion_optimizer_.goal_geom_;
  motion_optimizer_.goal_geom_ = RosHelpers::RosToXpp(msg.goal);
  motion_optimizer_.SetMotionType(static_cast<opt::MotionTypeID>(msg.motion_type));
  solver_type_ = msg.use_solver_snopt ? opt::Snopt : opt::Ipopt;

  if (goal_prev != motion_optimizer_.goal_geom_ || msg.motion_type_change) {
    OptimizeMotion();
  }




  // the forces generated for RA-L paper.
  if (msg.replay_trajectory) {

    static int i=0;

    std::vector<double> x_vel = { 1.3,   1.4,  0.1,  -1.4, -0.8,  0.8,  1.9   };
    std::vector<double> y_vel = { -0.4,  0.05, 1.5,   0.0, -1.5,  1.5,  0.0   };

    auto base = motion_optimizer_.opt_start_state_.GetBase();
    base.lin.v.x() = x_vel.at(i);//1.7;
    base.lin.v.y() = y_vel.at(i);//1.7;
    i++;

    if (i>x_vel.size()-1) {
      i = 0;
    }

    motion_optimizer_.opt_start_state_.SetBase(base);
    OptimizeMotion();
    PublishTrajectory(true);


    // receiving complete body state
    PoseMsg hyq_pose_msg_;
    hyq_pose_msg_.header.frame_id = "world";

    auto msg_body = RosHelpers::XppToRos(base);

    hyq_pose_msg_.pose = msg_body.pose;
    double alpha = atan2( base.lin.v.y(),base.lin.v.x());


    hyq_pose_msg_.pose.position.x -= cos(alpha)*0.8;
    hyq_pose_msg_.pose.position.y -= sin(alpha)*0.8;

    hyq_pose_msg_.pose.orientation.x = 0;
    hyq_pose_msg_.pose.orientation.y = 0;
    hyq_pose_msg_.pose.orientation.z = sin(alpha/2);
    hyq_pose_msg_.pose.orientation.w = cos(alpha/2);


    // publishing only the pose part
    pub_.publish(hyq_pose_msg_);


//  ROS_INFO_STREAM("Goal state set to:\n" << motion_optimizer_.goal_cog_);
//  ROS_INFO_STREAM("Time left:" << msg.t_left);
  }
}

void
NlpOptimizerNode::PublishTrajectory (bool use_new_goal_as_start)
{
  auto opt_traj_cartesian = motion_optimizer_.GetTrajectory(dt_);

  // convert to joint angles
  auto opt_traj_joints = RobotState::BuildWholeBodyTrajectory(opt_traj_cartesian,
                                  std::make_shared<hyq::HyqInverseKinematics>());

  auto msg = RosHelpers::XppToRos(opt_traj_joints);
  trajectory_pub_.publish(msg);

  if (use_new_goal_as_start) {
    auto fk = std::make_shared<hyq::codegen::HyQKinematics>();
    motion_optimizer_.BuildOptimizationStartState(opt_traj_joints.back().ConvertToCartesian(fk));
  }
}

/** Checks if this executable is run from where the config files for the
  * solvers are.
  */
static bool CheckIfInDirectoyWithIpoptConfigFile()
{
  char cwd[1024];
  getcwd(cwd, sizeof(cwd));
  std::string path(cwd);

  std::string ipopt_config_dir("config");
  if (path.substr( path.length() - ipopt_config_dir.length() ) != ipopt_config_dir) {
    std::string error_msg;
    error_msg = "Not run in correct directory. ";
    error_msg += "This executable has to be run from xpp_opt/" + ipopt_config_dir;
    throw ::ros::Exception(error_msg);
    return false;
  }

  return true;
}

} /* namespace ros */
} /* namespace xpp */
