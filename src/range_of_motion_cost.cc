/**
 @file    range_of_motion_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/zmp/range_of_motion_cost.h>
#include <xpp/zmp/cost_container.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

typedef Eigen::VectorXd VectorXd;
typedef OptimizationVariables::StdVecEigen2d FootholdsXY;

RangeOfMotionCost::RangeOfMotionCost ()
{
}

void
RangeOfMotionCost::Init (const OptimizationVariablesInterpreter& interpreter)
{
  continuous_spline_container_ = interpreter.GetSplineStructure();
  supp_polygon_container_ = interpreter.GetSuppPolygonContainer();
}

void
RangeOfMotionCost::UpdateVariables(const CostContainer* container)
{
  VectorXd x_coeff      = container->GetSplineCoefficients();
  FootholdsXY footholds = container->GetFootholds();

  continuous_spline_container_.AddOptimizedCoefficients(x_coeff);
  supp_polygon_container_.SetFootholdsXY(footholds);
}

double
RangeOfMotionCost::EvaluateCost () const
{
  utils::StdVecEigen2d footholds_b, nominal_footholds_b;
  footholds_b = builder_.GetFeetInBase(continuous_spline_container_,
                                       supp_polygon_container_,
                                       nominal_footholds_b);
  VectorXd g(footholds_b.size()*utils::kDim2d);
  int i=0;
  for (const auto& vec2d : footholds_b) {
    g.middleRows(i, utils::kDim2d) = vec2d;
    i += utils::kDim2d;
  }

  return g.norm();
}

} /* namespace zmp */
} /* namespace xpp */
