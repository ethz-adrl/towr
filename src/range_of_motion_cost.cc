/**
 @file    range_of_motion_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/zmp/range_of_motion_cost.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

typedef Eigen::VectorXd VectorXd;
typedef xpp::utils::StdVecEigen2d FootholdsXY;

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
RangeOfMotionCost::UpdateVariables(const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(OptimizationVariables::kSplineCoeff);
  VectorXd footholds = opt_var->GetVariables(OptimizationVariables::kFootholds);

  continuous_spline_container_.AddOptimizedCoefficients(x_coeff);
  supp_polygon_container_.SetFootholdsXY(utils::ConvertEigToStd(footholds));
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
