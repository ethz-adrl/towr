/**
 @file    linear_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Implements a special form of constraint, namely linear.
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_LINEAR_CONSTRAINT_H_
#define XPP_OPT_INCLUDE_XPP_OPT_LINEAR_CONSTRAINT_H_

#include <xpp/matrix_vector.h>
#include <xpp/constraint.h>
#include <xpp/opt/base_motion.h>

namespace xpp {
namespace opt {

/** @brief Calculates the constraint violations for linear constraints.
  *
  * This class is responsible for getting the current state of the CoM spline
  * and using this to calculate the constraint violations.
  */
class LinearEqualityConstraint : public Constraint {
public:
  using ComMotionPtr = std::shared_ptr<BaseMotion>;

  /** @brief Defines the elements of the linear constraint as g = Mx+v.
    *
    * @param com Center of Mass parametrization, from which spline coefficients x are used.
    * @param linear_equation the matrix M and vector v.
    */
  LinearEqualityConstraint (const ComMotionPtr& com,
                            const MatVec& linear_equation,
                            const std::string& name);
  virtual ~LinearEqualityConstraint ();

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  void UpdateConstraintValues () override;
  void UpdateBounds () override;

private:
  ComMotionPtr com_motion_;
  MatVec linear_equation_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_LINEAR_CONSTRAINT_H_ */
