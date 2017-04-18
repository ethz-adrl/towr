/**
 @file    constraint_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides a class to combine knowledge of individual constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "bound.h"
#include "constraint.h"

namespace xpp {
namespace opt {

/** @brief Knows about all constraints and gives information about them.
  *
  * For every constraint that \c ConstraintContainer knows about, it will return
  * the constraint violations and the acceptable bounds.
  */
// spring_clean_ should also be a component in a composite design pattern,
// just like Constraint.
// see https://sourcemaking.com/design_patterns/composite
// zmp_ rename to composite
class ConstraintContainer {
public:
  using VectorXd = Eigen::VectorXd;
  using Jacobian = Constraint::Jacobian;
  using ConstraintPtr = std::shared_ptr<Constraint>;
  using ConstraitPtrVec = std::vector<ConstraintPtr>;

  ConstraintContainer ();
  virtual ~ConstraintContainer ();

  void ClearConstraints();
  void AddConstraint (ConstraitPtrVec constraint);


  VectorXd GetConstraintValues () const;
  Jacobian GetJacobian () const;
  VecBound GetBounds () const;
  void UpdateConstraints();

//  void PrintStatus(double tol) const;

private:
  void RefreshBounds ();
  ConstraitPtrVec constraints_;

  // zmp_ these should probably only exist in the leaf/true constraints
  VecBound bounds_;
  mutable Jacobian jacobian_; // zmp_ remove this mutable
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_CONTAINER_H_ */
