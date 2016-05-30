/**
 @file    a_linear_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Implements a special form of constraint, namely linear.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_LINEAR_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_LINEAR_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/zmp/i_observer.h>

#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

/** @brief Calculates the constraint violations for linear constraints.
  *
  * This class is responsible for getting the current state of the optimization
  * variables from the subject and using this to calculate the constraint
  * violations.
  */
class ALinearConstraint : public IObserver, public AConstraint {
public:
  typedef xpp::utils::MatVec MatVec;

  virtual ~ALinearConstraint () {}

  /** @brief Defines the elements of the linear constraint as g = Mx+v.
    *
    * @param linear_equation the matrix M and vector v.
    */
  void Init(const MatVec& linear_equation);

  /** @brief Updates the values of the optimization variables */
  void Update() override;

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd EvaluateConstraint () const override;

  /** @brief Returns an upper and lower bound for each constraint violation */
  VecBound GetBounds () const override;

protected:
  /** only allow child classes of this class to be instantiated */
  ALinearConstraint (OptimizationVariables& subject);
  Bound bound_;

private:
  OptimizationVariables* subject_;  ///< this variable holds the current state of optimization variables
  VectorXd x_coeff_;                ///< the current spline coefficients
  MatVec linear_equation_;
};

class LinearEqualityConstraint : public ALinearConstraint {
public:
  LinearEqualityConstraint (OptimizationVariables& subject);
};

class LinearInequalityConstraint : public ALinearConstraint {
public:
  LinearInequalityConstraint (OptimizationVariables& subject);
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_LINEAR_CONSTRAINT_H_ */
