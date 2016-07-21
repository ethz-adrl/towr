/*
 * range_of_motion_constraint.cc
 *
 *  Created on: May 26, 2016
 *      Author: winklera
 */

#include <xpp/zmp/range_of_motion_constraint.h>
#include <xpp/zmp/constraint_container.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

typedef xpp::utils::StdVecEigen2d FootholdsXY;

RangeOfMotionConstraint::RangeOfMotionConstraint ()
{
}

void
RangeOfMotionConstraint::Init (const OptimizationVariablesInterpreter& interpreter)
{
  continuous_spline_container_ = interpreter.GetSplineStructure();
  supp_polygon_container_ = interpreter.GetSuppPolygonContainer();
}

void
RangeOfMotionConstraint::UpdateVariables (const ConstraintContainer* container)
{
  VectorXd x_coeff      = container->GetSplineCoefficients();
  FootholdsXY footholds = container->GetFootholds();

  continuous_spline_container_.AddOptimizedCoefficients(x_coeff);
  // fixme move this to foothold class and generally see if i really need
  // the previous support polygon container, or if footholds + legs is enough
  // for sure need start stance
  for (uint i=0; i<footholds.size(); ++i)
    supp_polygon_container_.SetFootholdsXY(i,footholds.at(i).x(), footholds.at(i).y());
}

RangeOfMotionConstraint::VectorXd
RangeOfMotionConstraint::EvaluateConstraint () const
{
  utils::StdVecEigen2d footholds_b, nominal_footholds_b;
  footholds_b = builder_.GetFeetInBase(continuous_spline_container_,
                                       supp_polygon_container_,
                                       nominal_footholds_b);

  VectorXd g(footholds_b.size()*utils::kDim2d);
  int c=0;
  for (int i=0; i<footholds_b.size(); ++i) {
    g[c++] = footholds_b.at(i).x();
    g[c++] = footholds_b.at(i).y();
  }

  return g;
}

RangeOfMotionConstraint::VecBound
RangeOfMotionConstraint::GetBounds () const
{
  utils::StdVecEigen2d nominal_footholds_b;
  builder_.GetFeetInBase(continuous_spline_container_,supp_polygon_container_,nominal_footholds_b);

  std::vector<Bound> bounds;
//  VectorXd g = EvaluateConstraint();
  double radius_x = 0.15; //m
  double radius_y = 0.15; //m
//  Bound bound(-radius, +radius);
//
//  for (int i=0; i<g.rows(); ++i)
//    bounds.push_back(bound);


  // SMELL  clean this up
  for (int i=0; i<nominal_footholds_b.size(); ++i) {
    Bound x_bound(nominal_footholds_b.at(i).x()-radius_x, nominal_footholds_b.at(i).x()+radius_x);
    Bound y_bound(nominal_footholds_b.at(i).y()-radius_y, nominal_footholds_b.at(i).y()+radius_y);
    bounds.push_back(x_bound);
    bounds.push_back(y_bound);
  }

  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
