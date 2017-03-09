/**
 @file    optimization_visualizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that visualizes the optimization values using ROS.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_

#include <xpp/opt/i_visualizer.h>
#include <xpp/utils/state.h>
#include <xpp/opt/contact.h>

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <xpp/ros/marker_array_builder.h>
#include <xpp_msgs/ContactVector.h>

namespace xpp {
namespace ros {

/** @brief Visualizes the current values of the optimization variables.
  *
  * This class is responsible for getting the state of the optimizaton
  * variables and generating ROS messages for rviz to visualize. The \c observer_
  * is responsible for supplying the interpreted optimization variables and
  * \c msg_builder_ is responsible for the generation of the ROS messages.
  */
// spring_clean_ move to xpp_vis repo
class RosVisualizer : public xpp::opt::IVisualizer {
public:
  using State       = xpp::utils::StateLin2d;
  using VecContacts = std::vector<xpp::opt::Contact>;

  using TrajMsg         = MarkerArrayBuilder::TrajMsg;
  using ContactVecMsg   = xpp_msgs::ContactVector;

  RosVisualizer();
  virtual ~RosVisualizer ();

  /** @brief Send a message with topic "optimization_variables" out to rviz */
  void Visualize() const override;

private:
  ::ros::Publisher ros_publisher_optimized_;
  ::ros::Publisher ros_publisher_optimized_single_;

  MarkerArrayBuilder msg_builder_;

  TrajMsg::ConstPtr robot_traj_;
  ::ros::Subscriber sub_;
  void TrajectoryCallback (const TrajMsg::ConstPtr& traj_msg);

  ::ros::Subscriber contacts_sub_;
  void ContactsCallback (const ContactVecMsg& msg);
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZATION_VISUALIZER_H_ */
