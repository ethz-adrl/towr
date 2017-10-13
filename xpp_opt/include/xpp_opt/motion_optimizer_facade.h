/**
 @file    motion_optimizer_facade.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   The facade for the motion optimization, ROS independent
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <xpp_opt/solvers/composite.h>
#include <xpp_opt/solvers/nlp.h>

#include <xpp_opt/height_map.h>
#include <xpp_opt/models/robot_model.h>
#include <xpp_opt/optimization_parameters.h>
#include <xpp_states/endeffectors.h>
#include <xpp_states/robot_state_cartesian.h>
#include <xpp_states/state.h>



namespace xpp {


/** Simplified interface to the complete motion optimization framework.
  */
class MotionOptimizerFacade {
public:
  using MotionParametersPtr      = std::shared_ptr<OptimizationParameters>;
  using OptimizationVariablesPtr = std::shared_ptr<Composite>;
  using RobotStateVec            = std::vector<RobotStateCartesian>;

  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  void SetInitialState(const RobotStateCartesian&);

  enum NlpSolver { Ipopt, Snopt } nlp_solver_;
  void SolveProblem();
  std::vector<RobotStateVec> GetIntermediateSolutions(double dt) const;
  RobotStateVec GetTrajectory(double dt) const;

  State3dEuler inital_base_;

  HeightMap::Ptr terrain_;
  RobotModel model_;
  MotionParametersPtr params_;

//  const MotionParametersPtr GetMotionParameters() const { return params_;};

  void SetFinalState(const StateLin3d& lin, const StateLin3d& ang);
  void SetIntialFootholds(EndeffectorsPos pos) {initial_ee_W_ = pos; };

  void SetTerrainFromAvgFootholdHeight() const
  {
    double avg_height=0.0;
    for ( auto pos : initial_ee_W_.ToImpl())
      avg_height += pos.z()/initial_ee_W_.GetCount();
    terrain_->SetGroundHeight(avg_height);
  }

private:
  EndeffectorsPos initial_ee_W_;
  State3dEuler final_base_;
  RobotStateVec GetTrajectory(const OptimizationVariablesPtr&, double dt) const;

  void BuildDefaultInitialState();
  OptimizationVariablesPtr BuildVariables() const;
  void BuildCostConstraints(const OptimizationVariablesPtr&);

  void SetBaseRepresentationCoeff(OptimizationVariablesPtr&) const;
  void SetBaseRepresentationHermite(OptimizationVariablesPtr&) const;

  mutable NLP nlp;
};

} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
