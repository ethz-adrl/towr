/**
 @file    range_of_motion_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_RANGE_OF_MOTION_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_RANGE_OF_MOTION_COST_H_

#include <xpp/zmp/a_cost.h>
#include <xpp/zmp/foothold_nominal_deviation.h>

namespace xpp {
namespace zmp {

class OptimizationVariablesInterpreter;

class RangeOfMotionCost : public ACost {
public:
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  // refactor use com_motion instead of spline, more general
  typedef ComSpline::Ptr ComMotionPtr;

  RangeOfMotionCost ();
  virtual ~RangeOfMotionCost () {}

  void Init(const OptimizationVariablesInterpreter&);
  void UpdateVariables(const OptimizationVariables*) override;
  double EvaluateCost () const override;

private:
  FootholdNominalDeviation builder_;

  SupportPolygonContainer supp_polygon_container_;
  ComMotionPtr continuous_spline_container_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_RANGE_OF_MOTION_COST_H_ */
