/**
 @file    a_linear_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_LINEAR_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_LINEAR_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/zmp/i_observer.h>

#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

class ALinearConstraint : public IObserver, public AConstraint {
public:
  typedef xpp::utils::MatVec MatVec;

  virtual ~ALinearConstraint () {}

  void Init(const MatVec& linear_equation);
  void Update() override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

protected:
  /** only allow child classes of this class to be instantiated */
  ALinearConstraint (OptimizationVariables& subject);
  Bound bound_;

private:
  OptimizationVariables* subject_;
  VectorXd x_coeff_; ///< the current spline coefficients
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
