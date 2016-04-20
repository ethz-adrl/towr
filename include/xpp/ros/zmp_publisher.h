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
namespace ros {

class ZmpPublisher {

public:
  typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > StdVecEigen2d;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef visualization_msgs::Marker Marker;
  typedef visualization_msgs::MarkerArray MarkerArray;
  typedef Eigen::Vector2d Vector2d;

public:
  ZmpPublisher (const xpp::zmp::ContinuousSplineContainer& trajectory);
  virtual
  ~ZmpPublisher ();

public:

  void AddRvizMessage(
      const Eigen::VectorXd& opt_spline_coeff,
      const VecFoothold& opt_footholds,
      double gap_center_x,
      double gap_width_x,
      const std::string& rviz_namespace,
      double alpha = 1.0);

  void publish() const { ros_publisher_.publish(zmp_msg_); };

  visualization_msgs::MarkerArray zmp_msg_;
  void AddStartStance(MarkerArray& msg,
      const std::vector<xpp::hyq::Foothold>& start_stance,
      const std::string& rviz_namespace);
  void AddGoal(MarkerArray& msg,const Vector2d& goal);
  void AddPolygon(const std::vector<xpp::hyq::Foothold>& footholds,
                  xpp::hyq::LegID leg_id);
private:
  void AddTrajectory(visualization_msgs::MarkerArray& msg,
                     xpp::zmp::SplineContainer zmp_splines,
                     const std::vector<xpp::hyq::Foothold>& H_footholds,
                     const std::string& rviz_namespace,
                     double alpha = 1.0);

  void AddFootholds(
      visualization_msgs::MarkerArray& msg,
      const VecFoothold& H_footholds,
      const std::string& rviz_namespace,
      int32_t type = visualization_msgs::Marker::SPHERE,
      double alpha = 1.0);

  void AddLineStrip(visualization_msgs::MarkerArray& msg, double center_x, double width_x) const;
  Marker GenerateMarker(Vector2d pos, int32_t type, double size) const;

  std_msgs::ColorRGBA GetLegColor(xpp::hyq::LegID leg) const;

  const std::string frame_id_ = "world";
  xpp::zmp::ContinuousSplineContainer trajectory_;
  ::ros::Publisher ros_publisher_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_PUBLISHER_H_ */
