/*
 * optimizer_node_base.h
 *
 *  Created on: Apr 22, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp/hyq/foothold.h>
#include <xpp/zmp/spline_container.h>

// custom msgs
#include <xpp_opt/StateLin3d.h>
#include <keyboard/Key.h>

#include <ros/ros.h>

namespace xpp {
namespace ros {

class OptimizerNodeBase {
public:
  typedef xpp::utils::Point3d State;
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::hyq::Foothold Foothold;
  typedef std::vector<Foothold> VecFoothold;
  typedef xpp_opt::StateLin3d StateMsg;
  typedef xpp::zmp::SplineContainer::VecSpline VecSpline;

public:
  OptimizerNodeBase ();
  virtual
  ~OptimizerNodeBase ();

protected:
  ::ros::NodeHandle n_;
  State goal_cog_;
  State curr_cog_;
  VecFoothold curr_stance_;
  bool start_with_com_shift_; ///< whether to shift Center of Mass before lifting first leg

  VecSpline opt_splines_;
  VecFoothold footholds_;

  xpp::zmp::SplineTimes spline_times_;
  double robot_height_;

private:
  ::ros::Subscriber goal_state_sub_;
  ::ros::Subscriber goal_key_sub_;
  void GoalStateCallback(const StateMsg& msg);
  void GoalStateCallbackKeyboard(const keyboard::Key& msg);
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_ */
