/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_

#include <xpp/utils/state.h>
#include "variable_set.h"
#include "motion_structure.h"
#include "com_motion.h"
#include <memory>
#include "motion_parameters.h"

namespace xpp {
namespace opt {

class AConstraint;
class ACost;

/** Builds all types of constraints/costs for the user.
  *
  * Implements the factory method, hiding object creation from the client.
  * The client specifies which object it wants, and this class is responsible
  * for the object creation. Factory method is like template method pattern
  * for object creation.
  */
class CostConstraintFactory {
public:
  using ConstraintPtr = std::shared_ptr<AConstraint>;
  using CostPtr       = std::shared_ptr<ACost>;
  using Vector2d      = Eigen::Vector2d;
  using State2d       = xpp::utils::StateLin2d;
  using MotionTypePtr = std::shared_ptr<MotionParameters>;
  using ComMotionPtr  = std::shared_ptr<ComMotion>;

  CostConstraintFactory ();
  virtual ~CostConstraintFactory ();

  void Init(const ComMotionPtr&, const MotionStructure&, const MotionTypePtr& params);

  // optimization variables with initial values
 VariableSet SplineCoeffVariables();
 VariableSet ContactVariables(const Vector2d initial_pos);
 VariableSet ConvexityVariables();
 VariableSet CopVariables();

  // constraints
  ConstraintPtr InitialConstraint_(const State2d& init);
  ConstraintPtr FinalConstraint_(const State2d& final_state_xy);
  ConstraintPtr JunctionConstraint_();
  ConstraintPtr ConvexityConstraint_();
  ConstraintPtr SupportAreaConstraint_();
  ConstraintPtr DynamicConstraint_(double robot_height);
  ConstraintPtr RangeOfMotionBoxConstraint_();
  ConstraintPtr FinalStanceConstraint_(const Vector2d& goal_xy);
  ConstraintPtr ObstacleConstraint_();

  // costs
  CostPtr GetCost(CostName name);
  CostPtr CreateFinalComCost(const State2d& final_state_xy);
  CostPtr CreateFinalStanceCost(const Vector2d& goal_xy);

private:
  MotionStructure motion_structure;
  MotionTypePtr params;
  ComMotionPtr com_motion;

  CostPtr ComMotionCost_();
  CostPtr RangeOfMotionCost_();
  CostPtr PolygonCenterCost_();
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_ */
