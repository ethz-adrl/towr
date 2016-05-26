/**
 @file    spline_junction_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_SPLINE_JUNCTION_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_SPLINE_JUNCTION_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/zmp/i_observer.h>

#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

class SplineJunctionConstraint : public IObserver, public AConstraint {
public:
  typedef xpp::utils::MatVec MatVec;

  SplineJunctionConstraint (OptimizationVariables& subject);
  virtual ~SplineJunctionConstraint () {}

  void Init(const ContinuousSplineContainer&);
  void Update() override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

private:
  OptimizationVariables* subject_;
  MatVec lin_constraint_;
  VectorXd x_coeff_; ///< the current spline coefficients

  MatVec BuildLinearConstraint(const ContinuousSplineContainer& c);
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_SPLINE_JUNCTION_CONSTRAINT_H_ */
