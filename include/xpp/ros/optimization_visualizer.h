/**
 @file    optimization_visualizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that visualizes the optimization values using ROS.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_

#include <xpp/zmp/i_visualizer.h>
#include <xpp/utils/geometric_structs.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace xpp {

namespace zmp { class NlpObserver; }
namespace hyq { class Foothold; }

namespace ros {

/** @brief Visualizes the current values of the optimization variables.
  *
  * This class is responsible for getting the state of the optimizaton
  * variables and generating ROS messages for rviz to visualize. The \c observer_
  * is responsible for supplying the interpreted optimization variables and
  * \c msg_builder_ is responsible for the generation of the ROS messages.
  */
class OptimizationVisualizer : public xpp::zmp::IVisualizer {
public:
  typedef std::shared_ptr<xpp::zmp::NlpObserver> NlpObserverPtr;
  using State = xpp::utils::Point2d;
  using VecFoothold = std::vector<xpp::hyq::Foothold>;

  OptimizationVisualizer();
  virtual ~OptimizationVisualizer ();

  void SetObserver(const NlpObserverPtr&);

  /** @brief Send a message with topic "optimization_variables" out to rviz */
  void Visualize() const override;

  void VisualizeCurrentState(const State& curr, const VecFoothold& start_stance) const;

private:
  ::ros::Publisher ros_publisher_;
  NlpObserverPtr observer_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_ */
