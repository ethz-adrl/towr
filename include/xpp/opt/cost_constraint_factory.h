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
#include <memory>

namespace xpp {
namespace opt {

class AConstraint;
class ACost;
class ComMotion;
class MotionStructure;

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

  CostConstraintFactory ();
  virtual ~CostConstraintFactory ();


  // optimization variables with initial values
  static VariableSet CreateSplineCoeffVariables(const ComMotion&);
  static VariableSet CreateContactVariables(const MotionStructure&,const Vector2d initial_pos);
  static VariableSet CreateConvexityVariables(const MotionStructure&);
  static VariableSet CreateCopVariables(const MotionStructure&);


  // constraints
  static ConstraintPtr CreateInitialConstraint(const State2d& init, const ComMotion&);
  static ConstraintPtr CreateFinalConstraint(const State2d& final_state_xy, const ComMotion&);
  static ConstraintPtr CreateJunctionConstraint(const ComMotion&);
  static ConstraintPtr CreateConvexityContraint(const MotionStructure&);
  static ConstraintPtr CreateSupportAreaConstraint(const MotionStructure&);
  static ConstraintPtr CreateDynamicConstraint(const ComMotion&, const MotionStructure&,double robot_height);
  static ConstraintPtr CreateRangeOfMotionConstraint(const ComMotion&, const MotionStructure&);
  static ConstraintPtr CreateFinalStanceConstraint(const Vector2d& goal_xy, const MotionStructure&);
  static ConstraintPtr CreateObstacleConstraint();


  // costs
  static CostPtr CreateMotionCost(const ComMotion&, const xpp::utils::MotionDerivative);
  static CostPtr CreateRangeOfMotionCost(const ComMotion&, const MotionStructure&);
  static CostPtr CreatePolygonCenterCost(const MotionStructure&);
  static CostPtr CreateFinalComCost(const State2d& final_state_xy, const ComMotion&);
  static CostPtr CreateFinalStanceCost(const Vector2d& goal_xy, const MotionStructure&);
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_ */
