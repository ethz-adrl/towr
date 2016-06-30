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

#include <xpp_opt/OptimizedParametersNlp.h>
#include <xpp_opt/RequiredInfoNlp.h>
#include <xpp/hyq/hyq_spliner.h>
#include <xpp/zmp/spline_container.h>

#include <iit/robots/hyq/declarations.h>
#include <iit/robots/hyq/inertia_properties.h>
#include <iit/robots/hyq/jsim.h>
#include <iit/robots/hyq/transforms.h>

//#include "state_machine.h" // always sl includes last, because of #define macro
#include "virtual_model-inl.h"


#include <Eigen/src/Core/Matrix.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <vector>
#include "controller.h"


namespace xpp {
namespace exe {

class ZmpRunner;

//fixme, move to other class
class ControllerState {
public:
  virtual ~ControllerState() {};
//  explicit ControllerState();

  virtual void Run(ZmpRunner*) const = 0;
};

class Planning : public ControllerState {
public:
  void Run(ZmpRunner* context) const;
};

class Executing : public ControllerState {
public:
  void Run(ZmpRunner* context) const;
};


/**
\class ZmpRunner
\brief C++ representation of task that walks with respect to the zmp
 */
class ZmpRunner : public Controller {
public:
  typedef Eigen::Vector3d Vector3d;
  typedef iit::HyQ::JointState JointState;
  typedef xpp::hyq::HyqState HyqState;
  typedef xpp::hyq::HyqSpliner HyqSpliner;
  typedef xpp::hyq::Foothold Foothold;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef xpp::hyq::VirtualModel VirtualModel;
  template<typename T> using LegDataMap = xpp::hyq::LegDataMap<T>;
  typedef xpp::hyq::LegID LegID;
  typedef xpp::utils::Point3d State;
  typedef xpp::utils::Orientation Orientation;
  typedef xpp::zmp::SplineContainer::VecSpline VecSpline;
  // ROS stuff
  typedef xpp_opt::RequiredInfoNlp ReqInfoMsg;
  typedef xpp_opt::OptimizedParametersNlp OptimizedParametersMsg;

  explicit ZmpRunner();
  virtual ~ZmpRunner();

  void PlanTrajectory();
  bool ExecuteLoop();
  enum TaskState{EXECUTING, PLANNING, WAITING} state_;
  void SetState(TaskState state) {
    state_ = state;
  }
  bool first_time_sending_commands_;
private:
//  void AddVarForLogging();


  ///////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void InitDerivedClassMembers() override {
    state_ = PLANNING;
  }

  /*virtual*/ bool DoSomething() override
  {
    controller_state_.at(state_)->Run(this);
    return true;
//    switch (state_)
//    {
//      case PLANNING: {
//        PlanTrajectory();
//        state_ = EXECUTING;
//        break;
//      }
//      case EXECUTING: {
//        bool success = ExecuteLoop();
//        first_time_sending_commands_ = false;
//        if (!success)
//          state_ = PLANNING;
//        break;
//      }
//      case WAITING: {
//        break;
//      }
//
//    }
//    return true;
  }
///////////////////////////////////////////////////////////////////////////////

  std::map<TaskState, ControllerState*> controller_state_;


private:

  void OptParamsCallback(const OptimizedParametersMsg& msg);
  ::ros::Publisher current_info_pub_;
  ::ros::Subscriber opt_params_sub_;

  /** Estimates where the robot will be when optimization is complete in order
    * to start optimization from there.
    *
    * @param required_time
    * @return
    */
  State GetStartStateForOptimization(/*const double required_time*/) const;

  void EstimateCurrPose();

  bool reoptimize_before_finish_;


  VecSpline opt_splines_;
  VecFoothold opt_footholds_;


  HyqSpliner spliner_;  //for normal body, ori, and feet traj.
  HyqState P_des_;
  HyqState P_curr_;

  // some hacky stuff
  State prev_state_;
  double t_stance_initial_;
  double t_swing_;
  double robot_height_;
  double t_switch_; // when to abort spline and go to already next optimized one
  hyq::SplineNode switch_node_;
  double kOptTimeReq_;


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
};


} // namespace exe
} // namespace iit

#endif /* ZMP_ZMP_RUNNER */
