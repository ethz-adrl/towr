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

/** Simplified interface to the complete motion optimization framework
  *
  * This is ROS independent.
  */
// zmp_ possibly merge with NLP facade
class MotionOptimizerFacade {
public:
  using RobotStateVec            = std::vector<RobotStateCartesian>;
  using MotionParametersPtr      = std::shared_ptr<MotionParameters>;
  using OptimizationVariablesPtr = std::shared_ptr<OptimizationVariablesContainer>;

  using CostContainerPtr         = std::shared_ptr<CostContainer>;
  using ConstraintContainerPtr   = std::shared_ptr<ConstraintContainer>;
  using NLPPtr                   = std::shared_ptr<NLP>;


  MotionOptimizerFacade ();
  virtual ~MotionOptimizerFacade ();

  void OptimizeMotion(NlpSolver solver);
  RobotStateVec GetTrajectory(double dt);

  RobotStateCartesian start_geom_;
  StateLin3d goal_geom_;

  OptimizationVariablesPtr opt_variables_;

  void SetMotionParameters(const MotionParametersPtr& params);
  void BuildDefaultStartStance(const MotionParameters& params);

private:

  void SolveProblem(NlpSolver solver);
  void SolveNlp(NlpSolver solver);
  void SolveIpopt();
  void SolveSnopt();

//  NlpFacade nlp_facade_;
  MotionParametersPtr motion_parameters_;


  NLPPtr nlp_;
  CostContainerPtr costs_;
  ConstraintContainerPtr constraints_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
