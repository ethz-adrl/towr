/**
 @file    a_linear_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Implements a special form of constraint, namely linear.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_LINEAR_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_LINEAR_CONSTRAINT_H_

#include "a_constraint.h"
#include "matrix_vector.h"

namespace xpp {
namespace opt {

/** @brief Calculates the constraint violations for linear constraints.
  *
  * This class is responsible for getting the current state of the optimization
  * variables from the subject and using this to calculate the constraint
  * violations.
  */
class ALinearConstraint : public AConstraint {
public:
  /** @brief Defines the elements of the linear constraint as g = Mx+v.
    *
    * @param linear_equation the matrix M and vector v.
    */
  void Init(const MatVec& linear_equation, const std::string& name);

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd EvaluateConstraint () const override;

protected:
  /** only allow child classes of this class to be instantiated. */
  ALinearConstraint ();
  virtual ~ALinearConstraint () {}
  MatVec linear_equation_;
  VectorXd x_;                ///< the optimization variables
};


class LinearEqualityConstraint : public ALinearConstraint {
public:
  /** @brief Returns an upper and lower bound for each constraint violation. */
  VecBound GetBounds () const override;
};

class LinearInequalityConstraint : public ALinearConstraint {
public:
  /** @brief Returns an upper and lower bound for each constraint violation. */
  VecBound GetBounds () const override;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_LINEAR_CONSTRAINT_H_ */
