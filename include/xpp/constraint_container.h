/**
 @file    constraint_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides a class to combine knowledge of individual constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_

#include "a_constraint.h"
#include "i_observer.h"
#include "optimization_variables.h"
#include <memory>

namespace xpp {
namespace opt {

/** @brief Knows about all constraints and gives information about them.
  *
  * For every constraint that \c ConstraintContainer knows about, it will return
  * the constraint violations and the acceptable bounds. It also maintains a
  * connection to the optimization variables, and constantly keeps up-to-date
  * values of these (observer).
  */
class ConstraintContainer : public IObserver {
public:
  typedef AConstraint::VectorXd VectorXd;
  typedef AConstraint::Jacobian Jacobian;
  typedef std::shared_ptr<AConstraint> ConstraintPtr;
  typedef std::shared_ptr<Jacobian> JacobianPtr;

  ConstraintContainer (OptimizationVariables& subject);
  virtual ~ConstraintContainer ();

  void Update () override;
  void ClearConstraints();

  void AddConstraint (ConstraintPtr constraint);

  VectorXd EvaluateConstraints () const;
  JacobianPtr GetJacobian () const;
  VecBound GetBounds () const;

  void PrintStatus(double tol) const;

private:
  void RefreshBounds ();
  std::vector<ConstraintPtr> constraints_;
  VecBound bounds_;
  JacobianPtr jacobian_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_ */
