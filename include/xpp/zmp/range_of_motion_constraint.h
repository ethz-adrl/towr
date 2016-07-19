/*
 * range_of_motion_constraint.h
 *
 *  Created on: May 26, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/foothold_nominal_deviation.h>

namespace xpp {
namespace zmp {

class OptimizationVariablesInterpreter;

class RangeOfMotionConstraint : public AConstraint {
public:
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

  RangeOfMotionConstraint ();
  virtual ~RangeOfMotionConstraint () {};

  void Init(const OptimizationVariablesInterpreter&);
  void UpdateVariables(const ConstraintContainer*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

private:
  SupportPolygonContainer supp_polygon_container_;
  ContinuousSplineContainer continuous_spline_container_;

  FootholdNominalDeviation builder_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_ */
