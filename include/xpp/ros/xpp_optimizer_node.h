/*
 * xpp_optimizer_node.h
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_XPP_OPTIMIZER_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_XPP_OPTIMIZER_NODE_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp_opt/StateLin3d.h>
#include <xpp_opt/OptimizeTrajectory.h>
#include <xpp_opt/ReturnOptimizedTrajectory.h>

#include <xpp/zmp/nlp_optimizer.h>

#include <ros/ros.h>

namespace xpp {
namespace ros {

class XppOptimizerNode {
public:
  typedef xpp::zmp::SplineConstraints::State State;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::zmp::NlpOptimizer NlpOptimizer;
  typedef xpp::hyq::Foothold Foothold;

public:
  XppOptimizerNode ();
  virtual
  ~XppOptimizerNode ();


private:
  void CurrentStateCallback(const xpp_opt::StateLin3d& msg);
  void GoalStateCallback(const xpp_opt::StateLin3d& msg);

  State StateLinMsgTo2DState(const xpp_opt::StateLin3d& msg) const;
  void OptimizeTrajectory(VectorXd& opt_coefficients,
                          StdVecEigen2d& opt_footholds) const;
  std::vector<xpp::hyq::LegID> DetermineStepSequence() const;

  bool OptimizeTrajectoryService(xpp_opt::OptimizeTrajectory::Request& req,
                                 xpp_opt::OptimizeTrajectory::Response& res);

  bool ReturnOptimizedTrajectory(xpp_opt::ReturnOptimizedTrajectory::Request& req,
                                 xpp_opt::ReturnOptimizedTrajectory::Response& res);


  ::ros::Publisher opt_var_pub_;
  ::ros::Subscriber curr_state_sub_;
  ::ros::Subscriber goal_state_sub_;
  ::ros::ServiceServer service_;
  ::ros::ServiceServer return_trajectory_service_;


  State goal_cog_;
  State curr_cog_;
  xpp::hyq::LegDataMap<Foothold> curr_stance_;
  double curr_execution_time_;



  NlpOptimizer nlp_optimizer_;
  StdVecEigen2d opt_footholds_;
  VectorXd opt_coefficients_;

};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_XPP_OPTIMIZER_NODE_H_ */
