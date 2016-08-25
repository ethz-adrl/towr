/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINT_FACTORY_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINT_FACTORY_H_

#include <xpp/utils/geometric_structs.h>
#include <memory>

namespace xpp {
namespace zmp {

class AConstraint;
class ComSpline;
class OptimizationVariablesInterpreter;

/** Builds all types of constraints for the user.
  *
  * Implements the factory method, hiding object creation from the client.
  * The client specifies which object it want, and this class is responsible
  * for the object creation. Factory method is like template method pattern
  * for object creation.
  */
class ConstraintFactory {
public:
  typedef std::shared_ptr<AConstraint> ConstraintPtr;
  typedef std::shared_ptr<ComSpline> ComSplinePtr;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::utils::Point2d State2d;

  ConstraintFactory ();
  virtual ~ConstraintFactory ();

  static ConstraintPtr CreateAccConstraint(const Vector2d& init_acc_xy, const ComSplinePtr&);
  static ConstraintPtr CreateFinalConstraint(const State2d& final_state_xy, const ComSplinePtr&);
  static ConstraintPtr CreateJunctionConstraint(const ComSplinePtr&);
  static ConstraintPtr CreateZmpConstraint(const OptimizationVariablesInterpreter&);
  static ConstraintPtr CreateRangeOfMotionConstraint(const OptimizationVariablesInterpreter&);
  static ConstraintPtr CreateJointAngleConstraint(const OptimizationVariablesInterpreter&);
  static ConstraintPtr CreateObstacleConstraint();
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINT_FACTORY_H_ */
