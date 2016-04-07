/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#include <xpp/zmp/qp_optimizer.h>
#include <xpp/zmp/cost_function.h>
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
    :spline_structure_(spline_structure),
     zmp_constraint_(spline_structure),
     spline_constraint_(spline_structure)

{
  State final_state; // zero vel,acc,jerk
  final_state.p = supp_poly_container.GetCenterOfFinalStance();

  SetupQpMatrices(supp_poly_container, final_state, walking_height);
}


QpOptimizer::~QpOptimizer() {}


void QpOptimizer::SetupQpMatrices(
    const xpp::hyq::SupportPolygonContainer& supp_poly_container,
    const State& final_state,
    double walking_height)
{
  if (spline_structure_.splines_.empty()) {
    throw std::runtime_error("zmp.optimizer.cc: spline_info vector empty. First call ConstructSplineSequence()");
  }

  CostFunction cost_function(spline_structure_);
  cf_ = cost_function.CreateMinAccCostFunction();
  eq_ = CreateEqualityContraints(final_state);
  ineq_ = CreateInequalityContraints(walking_height, supp_poly_container);
}


QpOptimizer::MatVec
QpOptimizer::CreateEqualityContraints(const State &final_state) const
{
  return spline_constraint_.CreateSplineConstraints(final_state);
}


QpOptimizer::MatVec
QpOptimizer::CreateInequalityContraints(double walking_height,
                                        const xpp::hyq::SupportPolygonContainer& supp_poly_container) const
{
  MatVec zmp_x = spline_structure_.ExpressZmpThroughCoefficients(walking_height, xpp::utils::X);
  MatVec zmp_y = spline_structure_.ExpressZmpThroughCoefficients(walking_height, xpp::utils::Y);

  return zmp_constraint_.AddLineConstraints(zmp_x, zmp_y, supp_poly_container);
}


Eigen::VectorXd QpOptimizer::SolveQp()
{
  Eigen::VectorXd opt_spline_coeff_xy;

  double cost = Eigen::solve_quadprog(cf_.M, cf_.v,
                                      eq_.M.transpose(), eq_.v,
                                      ineq_.M.transpose(), ineq_.v,
                                      opt_spline_coeff_xy);

  if (cost == std::numeric_limits<double>::infinity() || cost < 0.002)
    throw std::length_error("Eigen::quadprog did not find a solution");

  return opt_spline_coeff_xy;
}


} // namespace zmp
} // namespace xpp
