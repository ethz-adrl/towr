/*
 * range_of_motion_constraint.h
 *
 *  Created on: May 26, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_

#include <xpp/zmp/i_observer.h>
#include <xpp/zmp/a_constraint.h>
#include <xpp/zmp/optimization_variables.h>

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/foothold_nominal_deviation.h>

namespace xpp {
namespace zmp {

class RangeOfMotionConstraint : public IObserver, public AConstraint {
public:
  typedef OptimizationVariables::StdVecEigen2d FootholdsXY;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

  RangeOfMotionConstraint (OptimizationVariables& subject);
  virtual ~RangeOfMotionConstraint () {};

  void Init(const ContinuousSplineContainer&, const SupportPolygonContainer&);
  void Update() override;
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
