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

#include "composite.h"
#include "centroidal_model.h"
#include "endeffectors.h"
#include "height_map.h"
#include "nlp.h"
#include "optimization_parameters.h"
#include "robot_state_cartesian.h"
#include "state.h"

namespace xpp {
namespace opt {


/** Simplified interface to the complete motion optimization framework.
  */
class MotionOptimizerFacade {
public:
  using MotionParametersPtr      = std::shared_ptr<OptimizationParameters>;
  using OptimizationVariablesPtr = std::shared_ptr<Composite>;
  using RobotStateVec            = std::vector<RobotStateCartesian>;

  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  enum NlpSolver { Ipopt, Snopt } nlp_solver_;
  void SolveProblem();
  std::vector<RobotStateVec> GetIntermediateSolutions(double dt) const;
  RobotStateVec GetTrajectory(double dt) const;

  EndeffectorsPos initial_ee_W_;
  State3dEuler inital_base_;
  State3dEuler final_base_;

  HeightMap::Ptr terrain_;
  CentroidalModel::Ptr model_;

  const MotionParametersPtr GetMotionParameters() const { return params_;};

private:
  RobotStateVec GetTrajectory(const OptimizationVariablesPtr&, double dt) const;

  void BuildDefaultInitialState();
  OptimizationVariablesPtr BuildVariables() const;
  void BuildCostConstraints(const OptimizationVariablesPtr&);

  void SetBaseRepresentationCoeff(OptimizationVariablesPtr&) const;
  void SetBaseRepresentationHermite(OptimizationVariablesPtr&) const;

  MotionParametersPtr params_;
  mutable NLP nlp;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
