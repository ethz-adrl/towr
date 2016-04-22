/*
 * xpp_optimizer_node.h
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_OPTIMIZER_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_OPTIMIZER_NODE_H_

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/zmp/qp_optimizer.h>

#include <xpp_opt/OptimizeTrajectory.h>
#include <xpp_opt/ReturnOptimizedTrajectory.h>


namespace xpp {
namespace ros {

class QpOptimizerNode : public OptimizerNodeBase {
public:
  typedef xpp::zmp::QpOptimizer QpOptimizer;

public:
  QpOptimizerNode ();
  virtual ~QpOptimizerNode () {};


private:
  void OptimizeTrajectory(VectorXd& opt_coefficients) const;

//  bool OptimizeTrajectoryService(xpp_opt::OptimizeTrajectory::Request& req,
//                                 xpp_opt::OptimizeTrajectory::Response& res);
//
//  bool ReturnOptimizedTrajectory(xpp_opt::ReturnOptimizedTrajectory::Request& req,
//                                 xpp_opt::ReturnOptimizedTrajectory::Response& res);
//
//
//  ::ros::ServiceServer service_;
//  ::ros::ServiceServer return_trajectory_service_;

  QpOptimizer qp_optimizer_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_OPTIMIZER_NODE_H_ */
