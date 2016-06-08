/**
 @file    optimization_visualizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that visualizes the optimization values using ROS.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_

// fixme remove these dependencies
#include <xpp/ros/i_visualizer.h>
#include <xpp/zmp/zmp_spline.h>
#include <xpp/hyq/foothold.h>

#include <xpp/ros/marker_array_builder.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

#include <xpp/zmp/interpreting_observer.h>

#include <ros/ros.h>

namespace xpp {
namespace ros {

/** @brief Visualizes the current values of the optimization variables.
  *
  * This class is responsible for getting the state of the optimizaton
  * variables and generating ROS messages for rviz to visualize. The \c observer_
  * is responsible for supplying the interpreted optimization variables and
  * \c msg_builder_ is responsible for the generation of the ROS messages.
  */
class OptimizationVisualizer : public xpp::ros::IVisualizer {
public:
  typedef std::vector<xpp::zmp::ZmpSpline> VecSpline;
  typedef std::vector<xpp::hyq::Foothold>VecFoothold;
  typedef std::shared_ptr<xpp::zmp::InterpretingObserver> InterpretingObserverPtr;

  OptimizationVisualizer();
  virtual ~OptimizationVisualizer () {}

  void SetObserver(const InterpretingObserverPtr&);
  void PublishMsg();

private:
  ::ros::Publisher ros_publisher_;
  MarkerArrayBuilder msg_builder_;

  InterpretingObserverPtr observer_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_ */
