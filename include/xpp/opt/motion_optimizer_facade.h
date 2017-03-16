/**
 @file    motion_optimizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   The facade for the motion optimization, ROS independent
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_

#include <xpp/opt/step_sequence_planner.h>
#include <xpp/opt/nlp_facade.h>
#include <xpp/robot_state_cartesian.h>
#include <xpp/opt/motion_parameters.h>
#include <xpp/opt/endeffectors_motion.h>
#include <xpp/opt/motion_phase.h>

namespace xpp {
namespace opt {

/** Simplified interface to the complete motion optimization framework
  *
  * This is ROS independent.
  */
// zmp_ possibly merge with NLP facade
class MotionOptimizerFacade {
public:
  using RobotStateVec = std::vector<RobotStateCartesian>;
  using MotionTypePtr = std::shared_ptr<MotionParameters>;
  using PhaseVec      = std::vector<MotionPhase>;
//  using ContactVec    = NlpFacade::ContactVec;

  using EEMotionPtrS = std::shared_ptr<EndeffectorsMotion>;

  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  void OptimizeMotion(NlpSolver solver);
  RobotStateVec GetTrajectory(double dt);
//  ContactVec GetContactVec(); // zmp_ remove this and in source

  void BuildOptimizationStartState(const RobotStateCartesian& curr_geom);

  RobotStateCartesian start_geom_;
  StateLin3d goal_geom_;
  EEMotionPtrS ee_motion_;

  void SetMotionType(const MotionTypePtr& motion_type);

private:
  NlpFacade nlp_facade_;
  StepSequencePlanner step_sequence_planner_;
  MotionTypePtr motion_type_;
  PhaseVec motion_phases_;

};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
