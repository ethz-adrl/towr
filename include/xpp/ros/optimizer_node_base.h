/**
 @file    optimizer_node_base.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Declares the class OptimizerNodeBase
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_

#include <xpp/utils/polynomial_helpers.h>
#include <xpp/utils/base_state.h>
#include <xpp/hyq/foothold.h>
#include <xpp_msgs/StateLin3d.h>
#include <xpp/opt/phase_info.h>

#include <ros/ros.h>

namespace xpp {
namespace ros {

// inv_dyn possibly remove entire node
class OptimizerNodeBase {
public:
  using State = xpp::utils::BaseLin3d;
  using VecFoothold = std::vector<xpp::hyq::Foothold>;
  using StateMsg = xpp_msgs::StateLin3d;
  using PhaseVec = xpp::opt::PhaseVec;
  using VecSpline = std::vector<xpp::utils::ComPolynomial>;

  OptimizerNodeBase ();
  virtual ~OptimizerNodeBase ();

  virtual void OptimizeTrajectory() = 0;
  virtual void PublishOptimizedValues() const {};
  virtual void PublishTrajectory() const = 0;

protected:
//  ::ros::NodeHandle n_;
  State goal_cog_;

  double t_swing_;
  double t_stance_initial_;
  double des_robot_height_;

private:
  ::ros::Subscriber goal_state_sub_;
  void GoalStateCallback(const StateMsg& msg);
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_ */
