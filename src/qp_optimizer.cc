/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#include <xpp/zmp/qp_optimizer.h>

#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/cost_function.h>
#include <xpp/zmp/spline_constraints.h>
#include <xpp/zmp/eigen_quadprog-inl.h>

#include <cmath>      // std::numeric_limits

namespace xpp {
namespace zmp {


QpOptimizer::QpOptimizer()
{
  std::cerr << "QP optimizer not properly initialized!\n";
}

QpOptimizer::QpOptimizer(const ContinuousSplineContainer& spline_structure,
                         const xpp::hyq::SupportPolygonContainer& supp_poly_container,
                         double walking_height)
{
  cost_function_ = CostFunction::CreateMinAccCostFunction(spline_structure);

  SplineConstraints::State final_state; // zero vel,acc,jerk
  final_state.p = supp_poly_container.GetCenterOfFinalStance();
  std::cout << "final_state.p: " << final_state.p;
  SplineConstraints spline_constraint(spline_structure);
  equality_constraints_ = spline_constraint.CreateAllSplineConstraints(final_state);

  ZmpConstraint zmp_constraint(spline_structure, walking_height);
  inequality_constraints_ = zmp_constraint.CreateLineConstraints(supp_poly_container);
}


Eigen::VectorXd QpOptimizer::SolveQp()
{
  Eigen::VectorXd opt_spline_coeff_xy;

  double cost = Eigen::solve_quadprog(
      cost_function_.M, cost_function_.v,
      equality_constraints_.M.transpose(), equality_constraints_.v,
      inequality_constraints_.M.transpose(), inequality_constraints_.v,
      opt_spline_coeff_xy);

  if (cost == std::numeric_limits<double>::infinity())
    throw std::length_error("Eigen::quadprog did not find a solution");

  return opt_spline_coeff_xy;
}


} // namespace zmp
} // namespace xpp
