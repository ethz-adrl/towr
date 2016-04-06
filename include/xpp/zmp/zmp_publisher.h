/*
 * zmp_publisher.h
 *
 *  Created on: Apr 5, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_PUBLISHER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_PUBLISHER_H_

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/foothold.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace xpp {
namespace zmp {

class ZmpPublisher {

public:
  typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > StdVecEigen2d;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;

public:
  ZmpPublisher (const xpp::zmp::ContinuousSplineContainer& trajectory);
  virtual
  ~ZmpPublisher ();

public:

  void AddRvizMessage(
      const Eigen::VectorXd& opt_spline_coeff,
      const VecFoothold& opt_footholds,
      const std::string& rviz_namespace,
      double alpha = 1.0);

  void publish() const { ros_publisher_.publish(zmp_msg_); };

  visualization_msgs::MarkerArray zmp_msg_;
private:
  void AddTrajectory(visualization_msgs::MarkerArray& msg,
                     xpp::zmp::SplineContainer zmp_splines,
                     const std::string& rviz_namespace,
                     double alpha = 1.0);

  void AddFootholds(
      visualization_msgs::MarkerArray& msg,
      const VecFoothold& H_footholds,
      const std::string& rviz_namespace,
      int32_t type = visualization_msgs::Marker::SPHERE,
      double alpha = 1.0);

  void AddLineStrip(visualization_msgs::MarkerArray& msg, double center_x, double width_x) const;

  const std::string frame_id_ = "world";
  xpp::zmp::ContinuousSplineContainer trajectory_;
  ros::Publisher ros_publisher_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_PUBLISHER_H_ */
