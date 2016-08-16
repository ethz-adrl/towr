/*
 * range_of_motion_constraint.cc
 *
 *  Created on: May 26, 2016
 *      Author: winklera
 */

#include <xpp/zmp/range_of_motion_constraint.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

typedef xpp::utils::StdVecEigen2d FootholdsXY;
typedef Eigen::Vector2d Vector2d;

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
RangeOfMotionConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(OptimizationVariables::kSplineCoeff);
  VectorXd footholds = opt_var->GetVariables(OptimizationVariables::kFootholds);

  continuous_spline_container_.AddOptimizedCoefficients(x_coeff);
  supp_polygon_container_.SetFootholdsXY(utils::ConvertEigToStd(footholds));
}

RangeOfMotionConstraint::VectorXd
RangeOfMotionConstraint::EvaluateConstraint () const
{
  utils::StdVecEigen2d B_r_baseToFeet, B_r_baseToNominal;
  B_r_baseToFeet = builder_.GetFeetInBase(continuous_spline_container_,
                                       supp_polygon_container_,
                                       B_r_baseToNominal);


  std::vector<double> g_vec;

  for (int i=0; i<B_r_baseToFeet.size(); ++i) {

//    // for the first time discretization, the footholds (start stance) as well
//    // as the body position is fixed, so no constraint must be added there.
//    static const int n_contacts_first_node = 4;
//    if (i<n_contacts_first_node)
//      continue; // the initial body position and footholds are fixed anyway
//
//    Vector2d B_r_footToNominal = -B_r_baseToFeet.at(i) + B_r_baseToNominal.at(i);
//
//    // circle constraint on feet (nonlinear constraint (squared))
//    g_vec.push_back(B_r_footToNominal.norm());

    // squared constraints on feet (sometimes better convergence)
    g_vec.push_back(B_r_baseToFeet.at(i).x());
    g_vec.push_back(B_r_baseToFeet.at(i).y());
  }

  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

RangeOfMotionConstraint::VecBound
RangeOfMotionConstraint::GetBounds () const
{
  std::vector<Bound> bounds;

//  // for circular bounds on footholds
//  VectorXd g = EvaluateConstraint();
//  double radius = 0.15; //m
//  Bound bound = AConstraint::kNoBound_;
//  bound.upper_ = radius;
//  for (int i=0; i<g.rows(); ++i)
//    bounds.push_back(bound);


  // if using "linear" bounds
  utils::StdVecEigen2d nominal_footholds_b;
  builder_.GetFeetInBase(continuous_spline_container_,supp_polygon_container_,nominal_footholds_b);
  double radius_x = 0.15; //m
  double radius_y = 0.15; //m
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
