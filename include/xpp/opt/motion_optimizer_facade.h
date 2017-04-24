/**
 @file    motion_optimizer_facade.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   The facade for the motion optimization, ROS independent
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_

#include <memory>
#include <vector>

#include <xpp/robot_state_cartesian.h>
#include <xpp/state.h>

#include <xpp/nlp.h>
#include <xpp/opt/constraints/composite.h>
#include "motion_parameters.h"

namespace xpp {
namespace opt {

enum NlpSolver { Ipopt, Snopt };

/** Simplified interface to the complete motion optimization framework.
  */
class MotionOptimizerFacade {
public:
  using MotionParametersPtr      = std::shared_ptr<MotionParameters>;
  using OptimizationVariablesPtr = std::shared_ptr<Composite>;
  using RobotStateVec            = std::vector<RobotStateCartesian>;

  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  void SolveProblem(NlpSolver solver);
  RobotStateVec GetTrajectory(double dt) const;

  RobotStateCartesian start_geom_;
  StateLin3d goal_geom_;

  void SetMotionParameters(const MotionParametersPtr& params);
  void BuildDefaultStartStance();

private:
  void BuildVariables();

  OptimizationVariablesPtr opt_variables_;
  MotionParametersPtr motion_parameters_;
  NLP nlp;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
