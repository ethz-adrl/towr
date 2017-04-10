/**
 @file    constraint_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides a class to combine knowledge of individual constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_

#include "constraint.h"
#include <memory>
#include "optimization_variables_container.h"

namespace xpp {
namespace opt {

/** @brief Knows about all constraints and gives information about them.
  *
  * For every constraint that \c ConstraintContainer knows about, it will return
  * the constraint violations and the acceptable bounds. It also maintains a
  * connection to the optimization variables, and constantly keeps up-to-date
  * values of these.
  */
class ConstraintContainer {
public:
  typedef Constraint::VectorXd VectorXd;
  typedef Constraint::Jacobian Jacobian;
  typedef std::shared_ptr<Jacobian> JacobianPtr;
  typedef std::shared_ptr<Constraint> ConstraintPtr;
  using ConstraitPtrVec = std::vector<ConstraintPtr>;

  ConstraintContainer (OptimizationVariablesContainer& subject);
  virtual ~ConstraintContainer ();

  void ClearConstraints();

  void AddConstraint (ConstraitPtrVec constraint);

  VectorXd EvaluateConstraints () const;
  JacobianPtr GetJacobian () const;
  VecBound GetBounds () const;

  void UpdateConstraints();

  void PrintStatus(double tol) const;

private:
  OptimizationVariablesContainer* opt_variables_;
  void RefreshBounds ();
  std::vector<ConstraintPtr> constraints_;
  VecBound bounds_;
  JacobianPtr jacobian_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_ */
