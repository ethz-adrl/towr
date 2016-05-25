/*
 * constraint_container.h
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINT_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINT_CONTAINER_H_

#include <xpp/zmp/a_constraint.h>

namespace xpp {
namespace zmp {

class ConstraintContainer {
public:
  typedef AConstraint::VectorXd VectorXd;
  typedef AConstraint::VecBound VecBound;

  ConstraintContainer ();
  virtual ~ConstraintContainer ();

  void AddConstraint (const AConstraint& constraint);

  VectorXd EvaluateConstraints ();
  VecBound GetBounds ();

private:
  std::vector<const AConstraint*> constraints_;
  VectorXd g_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINT_CONTAINER_H_ */
