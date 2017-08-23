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

#include <xpp/composite.h>
#include <xpp/endeffectors.h>
#include <xpp/robot_state_cartesian.h>
#include <xpp/state.h>

#include "motion_parameters.h"
#include "nlp.h"

namespace xpp {
namespace opt {


/** Simplified interface to the complete motion optimization framework.
  */
class MotionOptimizerFacade {
public:
  using MotionParametersPtr      = std::shared_ptr<MotionParameters>;
  using OptimizationVariablesPtr = std::shared_ptr<Composite>;
  using RobotStateVec            = std::vector<RobotStateCartesian>;
  using NLPIterations            = std::vector<RobotStateVec>;

  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  void SolveProblem(NlpSolver solver);
  NLPIterations GetTrajectories(double dt) const;
  RobotStateVec BuildTrajectory(const OptimizationVariablesPtr&,
                                const MotionParametersPtr&,
                                double dt) const;

  EndeffectorsPos initial_ee_W_;
  State3dEuler inital_base_;
  State3dEuler final_base_;

  const MotionParametersPtr GetMotionParameters() const { return params_;};

private:
  void BuildDefaultInitialState();
  OptimizationVariablesPtr BuildVariables() const;
  void BuildCostConstraints(const OptimizationVariablesPtr&);

  MotionParametersPtr params_;
  mutable NLP nlp;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
