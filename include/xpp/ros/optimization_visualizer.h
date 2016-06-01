/**
 @file    optimization_visualizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that visualizes the optimization values using ROS.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_

#include <xpp/zmp/a_observer_visualizer.h>
#include <xpp/zmp/zmp_spline.h>
#include <xpp/hyq/foothold.h>

#include <xpp/ros/marker_array_builder.h>
#include <xpp/zmp/optimization_variables.h>

#include <ros/ros.h>

namespace xpp {
namespace ros {

/** @brief Visualizes the current values of the optimization variables.
  *
  * This class is responsible for getting the current values of the optimization
  * variables and generating ROS messages for rviz to visualize. After adding
  * some semantic information to the optimization variables, it delegates
  * the actual generation of these messages to \c msg_builder_.
  */
class OptimizationVisualizer : public xpp::zmp::AObserverVisualizer {
public:
  typedef std::vector<xpp::zmp::ZmpSpline> VecSpline;
  typedef xpp::hyq::Foothold::VecFoothold VecFoothold;
  typedef xpp::zmp::OptimizationVariables OptimizationVariables;

  OptimizationVisualizer();
  OptimizationVisualizer (OptimizationVariables& subject);
  virtual ~OptimizationVisualizer () {}

  void RegisterWithSubject (OptimizationVariables& subject);

  /** @brief Updates the values of the optimization variables. */
   void Update() override;
  void PublishMsg();

private:

  void InitRos();
  OptimizationVariables* subject_;
  ::ros::Publisher ros_publisher_;
  MarkerArrayBuilder msg_builder_;

  // optimization variables
  VecSpline splines_;
  VecFoothold footholds_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_ */
