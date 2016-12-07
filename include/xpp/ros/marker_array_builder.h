/**
 @file    marker_array_builder.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that builds rviz markers
 */

#ifndef XPP_OPT_INCLUDE_MARKER_ARRAY_BUILDER_H_
#define XPP_OPT_INCLUDE_MARKER_ARRAY_BUILDER_H_

#include <xpp/opt/com_motion.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/hyq/foothold.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

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
  using VecFoothold     = std::vector<xpp::hyq::Foothold>;
  using ComMotion       = xpp::opt::ComMotion;
  using MotionStructure = xpp::opt::MotionStructure;
  using Vector2d        = Eigen::Vector2d;
  using Marker          = visualization_msgs::Marker ;
  using MarkerArray     = visualization_msgs::MarkerArray ;

public:
  MarkerArrayBuilder();
  virtual ~MarkerArrayBuilder () {};

  void AddStartStance(MarkerArray& msg,
                      const VecFoothold& start_stance) const;
  void AddPoint(MarkerArray& msg,
               const Vector2d& goal,
               std::string rviz_namespace,
               int marker_type);

  void AddSupportPolygons(MarkerArray& msg,
                          const MotionStructure&,
                          const VecFoothold& footholds) const;
  void BuildSupportPolygon(MarkerArray& msg,
                           const VecFoothold& stance_legs,
                           xpp::hyq::LegID leg_id) const;

  void AddCogTrajectory(MarkerArray& msg,
                        const ComMotion&,
                        const MotionStructure&,
                        const VecFoothold&,
                        const std::string& rviz_namespace,
                        double alpha = 1.0) const;

  void AddZmpTrajectory(MarkerArray& msg,
                      const ComMotion&,
                      const MotionStructure&,
                      double walking_height,
                      const VecFoothold&,
                      const std::string& rviz_namespace,
                      double alpha = 1.0) const;

  void AddFootholds(MarkerArray& msg,
                    const VecFoothold& H_footholds,
                    const std::string& rviz_namespace,
                    int32_t type = visualization_msgs::Marker::SPHERE,
                    double alpha = 1.0) const;

  void AddLineStrip(MarkerArray& msg,
                    double center_x, double width_x,
                    const std::string& rviz_namespace) const;

  void AddEllipse(MarkerArray& msg,
                  double center_x, double center_y,
                  double width_x, double width_y,
                  const std::string& rviz_namespace) const;
private:
  Marker GenerateMarker(Vector2d pos, int32_t type, double size) const;
  std_msgs::ColorRGBA GetLegColor(xpp::hyq::LegID leg) const;
  const std::string frame_id_ = "world";
};

} /* namespace ros */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_MARKER_ARRAY_BUILDER_H_ */
