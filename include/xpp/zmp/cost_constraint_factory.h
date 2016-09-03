/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_CONSTRAINT_FACTORY_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_CONSTRAINT_FACTORY_H_

#include <xpp/utils/geometric_structs.h>
#include <memory>

namespace xpp {

namespace hyq {
class SupportPolygonContainer;
}

namespace zmp {

class AConstraint;
class ACost;
class ComSpline;
class OptimizationVariablesInterpreter;

/** Builds all types of constraints/costs for the user.
  *
  * Implements the factory method, hiding object creation from the client.
  * The client specifies which object it wants, and this class is responsible
  * for the object creation. Factory method is like template method pattern
  * for object creation.
  */
class CostConstraintFactory {
public:
  typedef std::shared_ptr<AConstraint> ConstraintPtr;
  typedef std::shared_ptr<ComSpline> ComSplinePtr;
  typedef std::shared_ptr<ACost> CostPtr;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::utils::Point2d State2d;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

  CostConstraintFactory ();
  virtual ~CostConstraintFactory ();

  static ConstraintPtr CreateInitialConstraint(const State2d& init, const ComSplinePtr&);
  static ConstraintPtr CreateFinalConstraint(const State2d& final_state_xy, const ComSplinePtr&);
  static ConstraintPtr CreateJunctionConstraint(const ComSplinePtr&);
  static ConstraintPtr CreateZmpConstraint(const OptimizationVariablesInterpreter&);
  static ConstraintPtr CreateRangeOfMotionConstraint(const OptimizationVariablesInterpreter&);
//  static ConstraintPtr CreateJointAngleConstraint(const OptimizationVariablesInterpreter&);
  static ConstraintPtr CreateObstacleConstraint();

  static CostPtr CreateAccelerationCost(const ComSplinePtr&);
  static CostPtr CreateFinalComCost(const State2d& final_state_xy, const ComSplinePtr&);
  static CostPtr CreateRangeOfMotionCost(const OptimizationVariablesInterpreter&);
  static CostPtr CreateFinalStanceCost(const Vector2d& goal_xy, const SupportPolygonContainer& supp_polygon_container);
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_CONSTRAINT_FACTORY_H_ */
