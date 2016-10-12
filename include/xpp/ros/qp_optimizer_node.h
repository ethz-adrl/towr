/*
 * xpp_optimizer_node.h
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_OPTIMIZER_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_OPTIMIZER_NODE_H_

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/opt/qp_facade.h>

#include <xpp_msgs/RequiredInfoQp.h>        // receive
#include <xpp_msgs/OptimizedParametersQp.h> // send
#include <xpp_msgs/SolveQp.h>               // service

namespace xpp {
namespace ros {

class QpOptimizerNode : public OptimizerNodeBase {
public:
  typedef xpp::opt::QpFacade QpOptimizer;
  typedef xpp_msgs::RequiredInfoQp ReqInfoMsg;
  typedef xpp_msgs::OptimizedParametersQp OptParamMsg;

  QpOptimizerNode ();
  virtual ~QpOptimizerNode () {};

private:
  QpOptimizer qp_optimizer_;
  virtual void OptimizeTrajectory() override final;
  virtual void PublishOptimizedValues() const override final;
  virtual void PublishTrajectory() const override final {};
  void UpdateCurrentState(const ReqInfoMsg& msg);

  ::ros::Subscriber current_info_sub_;
  ::ros::Publisher opt_params_pub_;
  void CurrentInfoCallback(const ReqInfoMsg& msg);

  bool start_with_com_shift_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_FACADE_NODE_H_ */
