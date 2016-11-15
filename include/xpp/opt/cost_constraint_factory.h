/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_

#include <xpp/utils/base_state.h>
#include <memory>

namespace xpp {

namespace hyq {
class SupportPolygonContainer;
}

namespace opt {

class AConstraint;
class ACost;
class ComMotion;
class OptimizationVariablesInterpreter;
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
  typedef std::shared_ptr<AConstraint> ConstraintPtr;
  typedef std::shared_ptr<ACost> CostPtr;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::utils::BaseLin2d State2d;
  using Contacts = xpp::hyq::SupportPolygonContainer;

  CostConstraintFactory ();
  virtual ~CostConstraintFactory ();

  static ConstraintPtr CreateInitialConstraint(const State2d& init, const ComMotion&);
  static ConstraintPtr CreateFinalConstraint(const State2d& final_state_xy, const ComMotion&);
  static ConstraintPtr CreateJunctionConstraint(const ComMotion&);
  static ConstraintPtr CreateZmpConstraint(const MotionStructure&,
                                           const ComMotion&,
                                           const Contacts&,
                                           double walking_height,
                                           double dt_zmp);
  static ConstraintPtr CreateRangeOfMotionConstraint(const ComMotion&, const Contacts&,
                                                     const MotionStructure&);
  static CostPtr CreateRangeOfMotionCost(const ComMotion&, const Contacts&,
                                                     const MotionStructure&);

//  static ConstraintPtr CreateJointAngleConstraint(const OptimizationVariablesInterpreter&);
  static ConstraintPtr CreateObstacleConstraint(const Contacts&);

  static CostPtr CreateMotionCost(const ComMotion&, const xpp::utils::MotionDerivative);
  static CostPtr CreateFinalComCost(const State2d& final_state_xy, const ComMotion&);

  static CostPtr CreateFinalStanceCost(const Vector2d& goal_xy, const Contacts&);
  static ConstraintPtr CreateFinalStanceConstraint(const Vector2d& goal_xy, const Contacts&);
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_ */
