/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>
#include <xpp/zmp/com_spline.h>
#include <xpp/ros/ros_helpers.h>  // namespace cmo::ros

#include <xpp_msgs/ros_helpers.h> // namespace xpp::ros
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/HyqStateTrajectory.h>

#include <hyqb_msgs/Trajectory.h>


namespace xpp {
namespace ros {

using HyqTrajRvizMsg = hyqb_msgs::Trajectory;
static bool CheckIfInDirectoyWithIpoptConfigFile();

NlpOptimizerNode::NlpOptimizerNode ()
{
  current_info_sub_ = n_.subscribe(xpp_msgs::req_info_nlp,
                                   1, // take only the most recent information
                                   &NlpOptimizerNode::CurrentInfoCallback, this);

// opt_params_pub_ = n_.advertise<OptParamMsg>("optimized_parameters_nlp", 1);
// trajectory_pub_ = n_.advertise<RobotStateTrajMsg>(xpp_msgs::robot_trajectory, 1);

  // topic the urdf publisher subscribes to to send hyq trajectory to rviz
  std::string trajectory_topic = RosHelpers::GetStringFromServer("/hyq_rviz_trajectory_topic");
  trajectory_pub_rviz_ = n_.advertise<HyqTrajRvizMsg>(trajectory_topic, 1);


  trajectory_pub_hyqjoints_ = n_.advertise<xpp_msgs::HyqStateTrajectory>(
      xpp_msgs::robot_trajectory_joints, 1);

  supp_polygon_margins_ = xpp::hyq::SupportPolygon::GetDefaultMargins();
  supp_polygon_margins_[hyq::DIAG] = RosHelpers::GetDoubleFromServer("/xpp/margin_diag");

  max_cpu_time_ = RosHelpers::GetDoubleFromServer("/xpp/max_cpu_time");

  // get current optimization values from the optimizer
  optimization_visualizer_ = std::make_shared<OptimizationVisualizer>();
  optimization_visualizer_->SetObserver(nlp_facade_.GetObserver());
  nlp_facade_.AttachVisualizer(optimization_visualizer_);

  CheckIfInDirectoyWithIpoptConfigFile();

  whole_body_mapper_.SetParams(0.5, 0.15, 0.0);
  ROS_INFO_STREAM("Initialization done, ready to optimize!...");
}

void
NlpOptimizerNode::CurrentInfoCallback(const ReqInfoMsg& msg)
{
  UpdateCurrentState(msg);
//  OptimizeTrajectory();
//  PublishOptimizedValues();
//  optimization_visualizer_->Visualize();
}

void
NlpOptimizerNode::UpdateCurrentState(const ReqInfoMsg& msg)
{
  curr_cog_      = RosHelpers::RosToXpp(msg.curr_state);
  curr_stance_   = RosHelpers::RosToXpp(msg.curr_stance);
  curr_swingleg_ = msg.curr_swingleg;

  static bool first_update = true;
  if (first_update) {
    ROS_INFO_STREAM("Updated Current State: " << curr_cog_);
    ROS_INFO_STREAM("Updated Current Footholds: ");
    for (auto f : curr_stance_) {
      ROS_INFO_STREAM(f);
    }
    ROS_INFO_STREAM("Updated curr_swingleg: " << curr_swingleg_);
    first_update = false;
  }

  optimization_visualizer_->VisualizeCurrentState(curr_cog_.Get2D(), curr_stance_);
}

void
NlpOptimizerNode::PublishTrajectory () const
{
//  auto trajectory = whole_body_mapper_.BuildWholeBodyTrajectory();
//  RobotStateTrajMsg msg = xpp::ros::RosHelpers::XppToRos(trajectory);
//  trajectory_pub_.publish(msg);

  auto trajectory_hyq_joints = whole_body_mapper_.BuildWholeBodyTrajectoryJoints();
  auto msg_hyq_with_joints = xpp::ros::RosHelpers::XppToRosHyq(trajectory_hyq_joints);
  trajectory_pub_hyqjoints_.publish(msg_hyq_with_joints);

  HyqTrajRvizMsg msg_rviz = xpp::ros::RosHelpers::XppToRos(trajectory_hyq_joints);
  trajectory_pub_rviz_.publish(msg_rviz);

  optimization_visualizer_->Visualize(); // sends out the footholds and com motion to rviz
}

void
NlpOptimizerNode::OptimizeTrajectory()
{
  // cmo pull the "phase planner" out of the nlp facade
  nlp_facade_.SolveNlp(curr_cog_.Get2D(),
                       goal_cog_.Get2D(),
                       curr_swingleg_,
                       robot_height_,
                       curr_stance_,
                       supp_polygon_margins_,
                       t_swing_, t_stance_,
                       max_cpu_time_);

  auto& com_spline = dynamic_cast<xpp::zmp::ComSpline&>(*nlp_facade_.GetMotion());
  opt_splines_   = com_spline.GetPolynomials();
  footholds_     = nlp_facade_.GetFootholds();
  motion_phases_ = nlp_facade_.GetPhases();

  // convert to full body state
  whole_body_mapper_.Init(motion_phases_,opt_splines_,footholds_, robot_height_);
}


/** Checks if this executable is run from where the config files for the
  * solves are.
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
    error_msg += "This executable has to be run from ./" + ipopt_config_dir;
    throw ::ros::Exception(error_msg);
    return false;
  }

  return true;
}

//void
//NlpOptimizerNode::PublishOptimizedValues() const
//{
//  OptParamMsg msg_out;
//  msg_out.splines   = cmo::ros::RosHelpers::XppToRos(opt_splines_);
//  msg_out.footholds = xpp::ros::RosHelpers::XppToRos(footholds_);
//  msg_out.phases    = cmo::ros::RosHelpers::XppToRos(motion_phases_);
//
//  opt_params_pub_.publish(msg_out);
//  ROS_INFO_STREAM("Publishing optimized values");
//}

} /* namespace ros */
} /* namespace xpp */
