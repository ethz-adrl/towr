/*!
 * \file   zmp_runner.h
 * \author Alexander Winkler
 * \date   Oct 4, 2014
 * \brief  SL Task that executes a walking gait given a arbitrary sequence and
 *         position of footholds.
 *
 *         It uses the dynamic locomotion library \c xpp to find an optimal and
 *         dynamicaly stable body trajectory and executes this trjaectory using
 *         inverse dynamics coupled with a virtual model controller.
 */

#ifndef IIT_ZMP_RUNNER_H_
#define IIT_ZMP_RUNNER_H_

#include <xpp_controller/controller.h>

#include "walking_controller_state.h"
#include <xpp/hyq/hyq_state.h>

#include <xpp_opt/RequiredInfoNlp.h>               // send
#include <xpp_opt/RobotStateTrajectoryCartesian.h> // receive

#include "virtual_model-inl.h"

#include <ros/ros.h>

namespace xpp {
namespace exe {


class WalkingController : public Controller {
public:
  typedef Eigen::Vector3d Vector3d;
  typedef iit::HyQ::JointState JointState;
  typedef xpp::hyq::HyqState HyqState;
  typedef xpp::hyq::Foothold Foothold;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef xpp::hyq::VirtualModel VirtualModel;
  template<typename T> using LegDataMap = xpp::hyq::LegDataMap<T>;
  typedef xpp::hyq::LegID LegID;
  typedef xpp::utils::Point3d State;
  typedef xpp_opt::RequiredInfoNlp ReqInfoMsg;
  using RobotStateTrajMsg = xpp_opt::RobotStateTrajectoryCartesian;

  explicit WalkingController();
  virtual ~WalkingController();
  void SetState(WalkingControllerState::State state);
  // fsm callable functions
//  void PublishCurrentState();
  void IntegrateOptimizedTrajectory();
  void ExecuteLoop();
  void EstimateCurrPose();
  bool EndCurrentExecution();
  bool IsTimeToSendOutState() const;
  void PublishOptimizationStartState(); // sends out command to start NLP optimization

  bool optimal_trajectory_updated;

private:
  void GetReadyHook() override;
  bool RunHook() override;

  WalkingControllerState::State current_state_;
  WalkingControllerState::StatesMap states_map_;

  void TrajectoryCallback(const RobotStateTrajMsg& msg);
  ::ros::Publisher current_info_pub_;
  ::ros::Subscriber trajectory_sub_;
  std::vector<xpp::hyq::HyqStateStamped> optimized_trajectory_;
  int k = 0; // position along optimized trajectory;

  bool reoptimize_before_finish_;
  bool first_run_after_integrating_opt_trajectory_;

  HyqState P_des_;
  HyqState P_curr_;
  HyqState switch_node_;  // where the new trajectory starts

  // some hacky stuff
  State prev_state_;
  double t_stance_initial_;
  double t_swing_;
  double robot_height_;
  double max_cpu_time_;

  void SmoothTorquesAtContactChange(JointState& uff);
  LegDataMap<bool> prev_swingleg_;
  bool ffsplining_;
  double ffspliner_timer_;
  double ffspline_duration_;
  JointState uff_prev_;
  Eigen::Vector3d b_r_geomtocog; // tranform from geometric body center to center of gravity

  bool use_virtual_model_;
  VirtualModel vm_;
  iit::HyQ::ForceTransforms force_transforms_;
  iit::HyQ::dyn::InertiaProperties inertia_properties_;
  iit::HyQ::dyn::JSIM jsim_; // Joint-Space Inertia Matrix


  Eigen::Vector3d TransformBaseToProjectedFrame(const Eigen::Vector3d& B_r_btox,
                                                const xpp::utils::Pose& P_base_BtoP) const;
//  void AddVarForLogging();
};


} // namespace exe
} // namespace iit

#endif /* ZMP_ZMP_RUNNER */
