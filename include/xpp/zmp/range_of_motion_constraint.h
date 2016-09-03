/*
 * range_of_motion_constraint.h
 *
 *  Created on: May 26, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_

#include "a_constraint.h"
#include "foothold_nominal_deviation.h"
#include "stance_feet_calculator.h"
#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace zmp {

class OptimizationVariablesInterpreter;

class RangeOfMotionConstraint : public AConstraint {
public:
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef FootholdNominalDeviation::ComMotionPtr ComMotionPtr;
  using PosXY = Eigen::Vector2d;
  using LegID = xpp::hyq::LegID;

  RangeOfMotionConstraint ();
  virtual ~RangeOfMotionConstraint () {};

  void Init(const OptimizationVariablesInterpreter&);
  void UpdateVariables(const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  PosXY GetNominalPositionInBase(LegID leg) const;

  // remove these as well, they are already in stance feet calc
  SupportPolygonContainer supp_polygon_container_;
  ComMotionPtr com_motion_;

  FootholdNominalDeviation builder_; // remove this one

  StanceFeetCalculator stance_feet_cal_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_ */
