/*
 * final_state_constraint.h
 *
 *  Created on: May 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FINAL_STATE_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FINAL_STATE_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/zmp/i_observer.h>

#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

class FinalStateConstraint : public AConstraint, public IObserver {
public:
  typedef xpp::utils::MatVec MatVec;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::utils::Point2d State2d;

  FinalStateConstraint (OptimizationVariables& subject);
  virtual ~FinalStateConstraint () {};

  void Init(const State2d& final_xy,
                         const ContinuousSplineContainer&);

  void Update() override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

private:
  OptimizationVariables* subject_;
  MatVec lin_constraint_;
  VectorXd x_coeff_; ///< the current spline coefficients

  MatVec BuildLinearConstraint(const State2d& final_xy,
                               const ContinuousSplineContainer& c);

  // delete the copy and copy assignment operators
  FinalStateConstraint& operator=(const FinalStateConstraint&) = delete;
  FinalStateConstraint(const FinalStateConstraint&)            = delete;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FINAL_STATE_CONSTRAINT_H_ */
