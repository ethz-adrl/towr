/**
 @file    marker_array_builder.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that builds rviz markers
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_PUBLISHER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_PUBLISHER_H_

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/foothold.h>

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace xpp {
namespace ros {

/** @brief Builds ROS marker array messages that can be visualized in rviz.
  *
  * This class is responsible for converting trajectories and footholds into
  * beautiful rviz markers. It knows nothing about trajectory optimization or
  * optimization variables.
  */
class MarkerArrayBuilder {

public:
  typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > StdVecEigen2d;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef visualization_msgs::Marker Marker;
  typedef visualization_msgs::MarkerArray MarkerArray;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::zmp::SplineContainer SplineContainer;
  typedef xpp::zmp::SplineContainer::VecSpline VecSpline;

public:
  MarkerArrayBuilder();
  virtual ~MarkerArrayBuilder () {};

  MarkerArray BuildMsg(const VecSpline& splines,
                       const VecFoothold& opt_footholds,
                       double walking_height);

private:
  void AddGoal(MarkerArray& msg,const Vector2d& goal);

  void AddSupportPolygons(MarkerArray& msg,
                          const VecFoothold& start_stance,
                          const VecFoothold& footholds) const;
  visualization_msgs::Marker BuildSupportPolygon(MarkerArray& msg,
                                                 const VecFoothold& stance_legs,
                                                 xpp::hyq::LegID leg_id) const;
  void AddStartStance(MarkerArray& msg, const VecFoothold& start_stance);

  void AddCogTrajectory(MarkerArray& msg,
                     const VecSpline& splines,
                     const std::vector<xpp::hyq::Foothold>& H_footholds,
                     const std::string& rviz_namespace,
                     double alpha = 1.0);

  void AddZmpTrajectory(MarkerArray& msg,
                     const VecSpline& splines,
                     double walking_height,
                     const std::vector<xpp::hyq::Foothold>& H_footholds,
                     const std::string& rviz_namespace,
                     double alpha = 1.0);

  void AddFootholds(
      MarkerArray& msg,
      const VecFoothold& H_footholds,
      const std::string& rviz_namespace,
      int32_t type = visualization_msgs::Marker::SPHERE,
      double alpha = 1.0);

  void AddLineStrip(MarkerArray& msg,
                    double center_x, double width_x,
                    const std::string& rviz_namespace) const;
  Marker GenerateMarker(Vector2d pos, int32_t type, double size) const;

  std_msgs::ColorRGBA GetLegColor(xpp::hyq::LegID leg) const;

  const std::string frame_id_ = "world";
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_PUBLISHER_H_ */
