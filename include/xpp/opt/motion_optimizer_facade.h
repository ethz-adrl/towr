/**
 @file    motion_optimizer_facade.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   The facade for the motion optimization, ROS independent
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_

#include <xpp/nlp.h>
#include <xpp/robot_state_cartesian.h>
#include <xpp/opt/motion_parameters.h>

namespace xpp {
namespace opt {

enum NlpSolver { Ipopt, Snopt };

/** Simplified interface to the complete motion optimization framework.
  */
class MotionOptimizerFacade {
public:
  using RobotStateVec            = std::vector<RobotStateCartesian>;
  using MotionParametersPtr      = std::shared_ptr<MotionParameters>;
  using OptimizationVariablesPtr = std::shared_ptr<OptimizationVariablesContainer>;

  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  void SolveProblem(NlpSolver solver);
  RobotStateVec GetTrajectory(double dt);

  RobotStateCartesian start_geom_;
  StateLin3d goal_geom_;

  void SetMotionParameters(const MotionParametersPtr& params);
  void BuildDefaultStartStance(const MotionParameters& params);

private:
  void BuildVariables();

  OptimizationVariablesPtr opt_variables_;
  MotionParametersPtr motion_parameters_;
  NLP nlp;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
