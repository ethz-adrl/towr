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
#include <hyqb_msgs/Trajectory.h>

namespace xpp {
namespace ros {

using HyqTrajRvizMsg = hyqb_msgs::Trajectory;

NlpOptimizerNode::NlpOptimizerNode ()
{
  current_info_sub_ = n_.subscribe(xpp_msgs::req_info_nlp,
                                   1, // take only the most recent information
                                   &NlpOptimizerNode::CurrentInfoCallback, this);

//  opt_params_pub_ = n_.advertise<OptParamMsg>("optimized_parameters_nlp", 1);
  trajectory_pub_ = n_.advertise<RobotStateTrajMsg>(xpp_msgs::robot_trajectory, 1);

  // inv_kin hardcode this or put in topic_names.h
  std::string trajectoryTopic;
  ::ros::param::param<std::string>("trajectory_topic", trajectoryTopic, std::string("/trajectory"));
  trajectory_pub_rviz_ = n_.advertise<HyqTrajRvizMsg>(trajectoryTopic, 10);

  supp_polygon_margins_ = xpp::hyq::SupportPolygon::GetDefaultMargins();
  supp_polygon_margins_[hyq::DIAG] = RosHelpers::GetDoubleFromServer("/xpp/margin_diag");

  max_cpu_time_ = RosHelpers::GetDoubleFromServer("/xpp/max_cpu_time");

  // get current optimization values from the optimizer
  optimization_visualizer_ = std::make_shared<OptimizationVisualizer>();
  optimization_visualizer_->SetObserver(nlp_facade_.GetObserver());
  nlp_facade_.AttachVisualizer(optimization_visualizer_);

  whole_body_mapper_.SetParams(0.5, 0.15, 0.0);
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
//  ROS_INFO_STREAM("Updated Current State: " << curr_cog_);

  optimization_visualizer_->VisualizeCurrentState(curr_cog_.Get2D(), curr_stance_);
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

void
NlpOptimizerNode::PublishTrajectory () const
{
  auto trajectory = whole_body_mapper_.BuildWholeBodyTrajectory();
  RobotStateTrajMsg msg = xpp::ros::RosHelpers::XppToRos(trajectory);
  trajectory_pub_.publish(msg);

  auto trajectory_rviz = whole_body_mapper_.BuildWholeBodyTrajectoryJoints();
  HyqTrajRvizMsg msg_rviz = xpp::ros::RosHelpers::XppToRos(trajectory_rviz);


//  // fill the trajectory
//  HyqTrajRvizMsg traj;
//  traj.dt.data = 0.01;
//  int n_states = 100;
//  traj.states.resize(n_states);
//  for (int i=0; i<n_states; ++i) {
//    traj.states.at(i).pose.position.x = 1.0*static_cast<double>(i)/n_states; // moves from zero to one meter forward
//    traj.states.at(i).pose.position.y = 0;
//    traj.states.at(i).pose.position.z = 0;
//    traj.states.at(i).pose.orientation.w = 1;
//    traj.states.at(i).joints.position.resize(12);
//    traj.states.at(i).joints.position.at(2) = -1.0;
//  }


  trajectory_pub_rviz_.publish(msg_rviz);
}

void
NlpOptimizerNode::OptimizeTrajectory()
{
  std::cout << "NlpOptimizerNode::OptimizeTrajector()..." << std::endl;

  optimization_visualizer_->ClearOptimizedMarkers();
  optimization_visualizer_->VisualizeCurrentState(curr_cog_.Get2D(), curr_stance_);

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

// cmo don't forget to put back in
  // convert to full body state
  whole_body_mapper_.Init(motion_phases_,opt_splines_,footholds_, robot_height_);
  std::cout << "finished init of whole body mapper" << std::endl;
//
  optimization_visualizer_->Visualize();
}

} /* namespace ros */
} /* namespace xpp */
