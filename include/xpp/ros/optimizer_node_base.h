/*
 * optimizer_node_base.h
 *
 *  Created on: Apr 22, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_

#include <xpp/zmp/phase_info.h>
#include <xpp/zmp/com_polynomial.h>
#include <xpp/utils/geometric_structs.h>

#include <xpp/hyq/foothold.h>
#include <xpp_opt/StateLin3d.h>

#include <ros/ros.h>

namespace xpp {
namespace ros {

class OptimizerNodeBase {
public:
  using State = xpp::utils::Point3d;
  using VecFoothold = std::vector<xpp::hyq::Foothold>;
  using StateMsg = xpp_opt::StateLin3d;
  using PhaseVec = xpp::zmp::PhaseVec;
  using VecSpline = std::vector<xpp::zmp::ComPolynomial>;

  OptimizerNodeBase ();
  virtual ~OptimizerNodeBase ();

protected:
  ::ros::NodeHandle n_;
  State goal_cog_;
  State curr_cog_;
  VecFoothold curr_stance_;

  VecSpline opt_splines_;
  VecFoothold footholds_;
  PhaseVec motion_phases_;

  double t_swing_;
  double t_stance_;
  double robot_height_;

private:
  ::ros::Subscriber goal_state_sub_;
  void GoalStateCallback(const StateMsg& msg);
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_ */
