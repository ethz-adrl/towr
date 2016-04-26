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

// custom msg and srv
#include <xpp_opt/StateLin3d.h>
#include <xpp_opt/ReturnOptSplines.h>
#include <xpp_opt/ReturnOptFootholds.h>

#include <ros/ros.h>

namespace xpp {
namespace ros {

class OptimizerNodeBase {
public:
  typedef xpp::utils::Point2d State;
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::hyq::Foothold Foothold;
  typedef std::vector<Foothold> VecFoothold;
  typedef xpp_opt::StateLin3d StateMsg;
  typedef xpp::zmp::SplineContainer::VecSpline VecSpline;

  typedef xpp_opt::ReturnOptSplines ReturnOptSplinesSrv;
  typedef xpp_opt::ReturnOptFootholds ReturnOptFootholds;

public:
  OptimizerNodeBase ();
  virtual
  ~OptimizerNodeBase ();


protected:
  ::ros::NodeHandle n_;
  State goal_cog_;
  State curr_cog_;
  xpp::hyq::LegDataMap<Foothold> curr_stance_;
//  double curr_execution_time_;

  VecSpline opt_splines_;
  VecFoothold footholds_;

private:
//  ::ros::Publisher opt_var_pub_;
//  ::ros::Subscriber curr_state_sub_;
  ::ros::Subscriber goal_state_sub_;
  ::ros::ServiceServer return_splines_srv_;
  ::ros::ServiceServer return_footholds_srv_;



  void CurrentStateCallback(const StateMsg& msg);
  void GoalStateCallback(const StateMsg& msg);
  // FIXME make these const
  bool ReturnOptimizedSplines(ReturnOptSplinesSrv::Request& req,
                              ReturnOptSplinesSrv::Response& res);
  bool ReturnOptimizedFootholds(ReturnOptFootholds::Request& req,
                                ReturnOptFootholds::Response& res);


};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_BASE_H_ */
