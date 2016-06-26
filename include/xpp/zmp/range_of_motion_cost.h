/**
 @file    range_of_motion_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_RANGE_OF_MOTION_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_RANGE_OF_MOTION_COST_H_

#include <xpp/zmp/a_cost.h>
#include <xpp/zmp/i_observer.h>
#include <xpp/zmp/optimization_variables.h>

#include <xpp/zmp/foothold_nominal_deviation.h>

namespace xpp {
namespace zmp {

class OptimizationVariablesInterpreter;

class RangeOfMotionCost : public ACost, public IObserver {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef OptimizationVariables::StdVecEigen2d FootholdsXY;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

  RangeOfMotionCost (OptimizationVariables& subject);
  virtual ~RangeOfMotionCost () {}

  void Init(const OptimizationVariablesInterpreter&);
  void Update() override;
  double EvaluateCost () const override;

private:
  FootholdNominalDeviation builder_;

  SupportPolygonContainer supp_polygon_container_;
  ContinuousSplineContainer continuous_spline_container_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_RANGE_OF_MOTION_COST_H_ */
