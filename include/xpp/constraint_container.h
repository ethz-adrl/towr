/**
 @file    constraint_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides a class to combine knowledge of individual constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "bound.h"
#include "constraint.h"

namespace xpp {
namespace opt {

/** @brief Knows about all constraints and gives information about them.
  *
  * For every constraint that \c ConstraintContainer knows about, it will return
  * the constraint violations and the acceptable bounds.
  */
class ConstraintContainer {
public:
  using VectorXd = Eigen::VectorXd;
  using Jacobian = Constraint::Jacobian;
  using JacobianPtr = std::shared_ptr<Jacobian>;
  using ConstraintPtr = std::shared_ptr<Constraint>;
  using ConstraitPtrVec = std::vector<ConstraintPtr>;

  ConstraintContainer ();
  virtual ~ConstraintContainer ();

  void ClearConstraints();

  void AddConstraint (ConstraitPtrVec constraint);

  VectorXd EvaluateConstraints () const;
  JacobianPtr GetJacobian () const;
  VecBound GetBounds () const;

  void UpdateConstraints();

  void PrintStatus(double tol) const;

private:
  void RefreshBounds ();
  ConstraitPtrVec constraints_;
  VecBound bounds_;
  JacobianPtr jacobian_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_ */
