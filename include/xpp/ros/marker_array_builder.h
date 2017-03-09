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
#include <xpp/opt/robot_state_cartesian.h>

#include <visualization_msgs/MarkerArray.h>
#include <xpp/utils/eigen_std_conversions.h>
#include <Eigen/Dense>

#include <functional> //std::function

#include <ros/ros.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>

namespace xpp {
namespace ros {

/** @brief Builds ROS marker array messages that can be visualized in rviz.
  *
  * This class is responsible for converting trajectories and footholds into
  * beautiful rviz markers. It knows nothing about trajectory optimization or
  * optimization variables.
  */

// spring_clean_ move to xpp_vis repo
class MarkerArrayBuilder {
public:
  using XyPositions     = utils::StdVecEigen2d;
  using ContactVec      = std::vector<xpp::opt::Contact>;
  using ComMotion       = xpp::opt::ComMotion;
  using MotionStructure = xpp::opt::MotionStructure;
  using Vector2d        = Eigen::Vector2d;
  using PosXYZ          = Eigen::Vector3d;
  using Marker          = visualization_msgs::Marker ;
  using MarkerArray     = visualization_msgs::MarkerArray ;
  using EEID            = xpp::utils::EndeffectorID;

  using TrajMsg         = xpp_msgs::RobotStateCartesianTrajectory;
  using RobotCartTraj   = std::vector<opt::RobotStateCartesian>;
  using FctPtr          = const std::function<Eigen::Vector2d(const utils::StateLin3d&)>;


public:
  MarkerArrayBuilder();
  virtual ~MarkerArrayBuilder () {};

public:
  RobotCartTraj robot_traj_;


  // spring_clean_ remove this
  void AddStartStance(MarkerArray& msg,
                      const ContactVec& start_stance) const;

  void AddPoint(MarkerArray& msg,
               const Vector2d& goal,
               std::string rviz_namespace,
               int marker_type) const;

  void AddSupportPolygons(MarkerArray& msg,
                          const MotionStructure&,
                          const XyPositions& footholds) const;
  void BuildSupportPolygon(MarkerArray& msg,
                           const ContactVec& stance_legs,
                           EEID leg_id) const;

  // only use the robot_traj for information
  void AddStartStance(MarkerArray& msg) const;
  void AddSupportPolygons(MarkerArray& msg) const;

  // spring_clean_ remove this
  void AddBodyTrajectory(MarkerArray& msg,
                        const ComMotion&,
                        const PosXYZ offset_geom_to_com,
                        const MotionStructure&,
                        const std::string& rviz_namespace,
                        double alpha = 1.0) const;

  // spring_clean_ remove this
  void AddZmpTrajectory(MarkerArray& msg,
                      const ComMotion&,
                      const MotionStructure&,
                      double walking_height,
                      const std::string& rviz_namespace) const;


  // spring_clean_ make this private
  void AddTrajectory(MarkerArray& msg,
                     const std::string& rviz_namespace,
                     double dt,
                     double marker_size,
                     const FctPtr& Get2dValue) const;


  void AddBodyTrajectory(MarkerArray& msg) const;
  void AddZmpTrajectory(MarkerArray& msg) const;


  void AddFootholds(MarkerArray& msg,
                    const ContactVec& H_footholds,
                    const std::string& rviz_namespace,
                    int32_t type = visualization_msgs::Marker::SPHERE,
                    double alpha = 1.0) const;

  void AddFootholds(MarkerArray& msg) const;



  void AddPendulum(MarkerArray& msg,
                   const ComMotion&,
                   const MotionStructure&,
                   double walking_height,
                   const std::string& rviz_namespace,
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
  std_msgs::ColorRGBA GetLegColor(EEID leg) const;
  const std::string frame_id_ = "world";
};

} /* namespace ros */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_MARKER_ARRAY_BUILDER_H_ */
