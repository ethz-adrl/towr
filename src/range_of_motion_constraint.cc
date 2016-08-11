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
RangeOfMotionConstraint::UpdateVariables (const ConstraintContainer* container)
{
  VectorXd x_coeff      = container->GetSplineCoefficients();
  FootholdsXY footholds = container->GetFootholds();

  continuous_spline_container_.AddOptimizedCoefficients(x_coeff);
  supp_polygon_container_.SetFootholdsXY(footholds);
}

RangeOfMotionConstraint::VectorXd
RangeOfMotionConstraint::EvaluateConstraint () const
{
  utils::StdVecEigen2d B_r_baseToFeet, B_r_baseToNominal;
  B_r_baseToFeet = builder_.GetFeetInBase(continuous_spline_container_,
                                       supp_polygon_container_,
                                       B_r_baseToNominal);

  static const int n_contacts_first_node = 4;

  std::vector<double> g_vec;

  for (int i=0; i<B_r_baseToFeet.size(); ++i) {

    if (i<n_contacts_first_node)
      continue; // the initial body position and footholds are fixed anyway

    Vector2d B_r_footToNominal = -B_r_baseToFeet.at(i) + B_r_baseToNominal.at(i);

    // circle constraint on feet (nonlinear constraint (squared))
    g_vec.push_back(B_r_footToNominal.norm());

//    g[c++] = B_r_baseToFeet.at(i).x();
//    g[c++] = B_r_baseToFeet.at(i).y();
  }

  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

RangeOfMotionConstraint::VecBound
RangeOfMotionConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  VectorXd g = EvaluateConstraint();
  std::cout << "g.size(): " << g.size() << std::endl;

  double radius = 0.15; //m
  Bound bound(0, +radius);

  for (int i=0; i<g.rows(); ++i)
    bounds.push_back(bound);


//  // SMELL  clean this up
//  utils::StdVecEigen2d nominal_footholds_b;
//  builder_.GetFeetInBase(continuous_spline_container_,supp_polygon_container_,nominal_footholds_b);
//  double radius_x = 0.15; //m
//  double radius_y = 0.15; //m
//  for (int i=0; i<nominal_footholds_b.size(); ++i) {
//    Bound x_bound(nominal_footholds_b.at(i).x()-radius_x, nominal_footholds_b.at(i).x()+radius_x);
//    Bound y_bound(nominal_footholds_b.at(i).y()-radius_y, nominal_footholds_b.at(i).y()+radius_y);
//    bounds.push_back(x_bound);
//    bounds.push_back(y_bound);
//  }

  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
