/*
 * initial_acceleration_constraint.h
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INITIAL_ACCELERATION_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INITIAL_ACCELERATION_CONSTRAINT_H_

#include <xpp/zmp/i_observer.h>
#include <xpp/zmp/a_constraint.h>


#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

class InitialAccelerationConstraint : public IObserver,
                                      public AConstraint {
public:
  typedef xpp::utils::MatVec MatVec;
  typedef Eigen::Vector2d Vector2d;

  InitialAccelerationConstraint (OptimizationVariables* subject);
  virtual ~InitialAccelerationConstraint () {};

  void SetDesiredInitialAcceleration(const Vector2d& acc_xy);

  void Update() override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

private:
  MatVec BuildLinearConstraint (const Vector2d& initial_acc);
  MatVec lin_constraint_;
  OptimizationVariables* subject_;


  VectorXd x_coeff_; ///< the current spline coefficients

  // delete the copy and copy assignment operators
  InitialAccelerationConstraint& operator=(const InitialAccelerationConstraint&) = delete;
  InitialAccelerationConstraint(const InitialAccelerationConstraint&)            = delete;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INITIAL_ACCELERATION_CONSTRAINT_H_ */
