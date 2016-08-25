/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#include <xpp/zmp/qp_facade.h>

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/linear_spline_equations.h>
#include <xpp/zmp/zmp_constraint_builder.h>
#include <xpp/zmp/com_spline4.h>
#include <xpp/zmp/com_spline6.h>
#include <xpp/utils/eigen_quadprog-inl.h>

#include <cmath>      // std::numeric_limits
// refactor don't have ros dependency here, this should only be in optimized node
#include <ros/console.h>

namespace xpp {
namespace zmp {

typedef xpp::utils::MatVecVec MatVecVec;

QpFacade::VecSpline
QpFacade::SolveQp(const State& initial_state,
                  const State& final_state,
                  const VecFoothold& start_stance,
                  const VecFoothold& steps,
                  const SplineTimes& times,
                  bool start_with_com_shift,
                  double robot_height)
{
  auto spline_structure = std::make_shared<ComSpline4>();
  spline_structure->Init(initial_state.p, initial_state.v , steps.size(), times,
                        start_with_com_shift);

//  auto spline_structure = std::make_shared<ComSpline6>();
//  spline_structure->Init(steps.size(), times, start_with_com_shift);

  LinearSplineEquations spline_eq(spline_structure);

  cost_function_ = spline_eq.MakeAcceleration(1.0,3.0);
  Eigen::MatrixXd reg = cost_function_.M;
  reg.setIdentity();

  // refactor see how this can be removed
//  cost_function_.M += 0.00001*reg; // because the matrix has to be positive semi definite.

  // refactor make this automatic depending on spline
  equality_constraints_ = MatVec(); // clear
  equality_constraints_ << spline_eq.MakeInitial(initial_state, {kAcc});
  equality_constraints_ << spline_eq.MakeFinal(final_state, {kPos, kVel, kAcc});
  equality_constraints_ << spline_eq.MakeJunction({kPos, kAcc});

  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  supp_polygon_container.Init(start_stance, steps, hyq::SupportPolygon::GetDefaultMargins());

  ROS_INFO_STREAM("Start polygon:\n" << supp_polygon_container.GetStartPolygon());
  std::vector<hyq::SupportPolygon> supp = supp_polygon_container.GetSupportPolygons();

  ROS_INFO_STREAM("Support polygons for the " << supp.size() << " steps:\n");
  for (const hyq::SupportPolygon& s : supp) {
    ROS_INFO_STREAM(s);
  }

  ZmpConstraintBuilder zmp_constraint(spline_structure, robot_height);
  MatVecVec zmp_constr = zmp_constraint.CalcZmpConstraints(supp_polygon_container);
  inequality_constraints_.M = zmp_constr.Mv.M;
  inequality_constraints_.v = zmp_constr.Mv.v + zmp_constr.constant;

//  std::cout << "zmp_constr.Mv.v" << zmp_constr.Mv.v.transpose() << std::endl;
//  std::cout << "zmp_constr.constant" << zmp_constr.constant.transpose() << std::endl;

  ROS_INFO_STREAM("Initial state:\t" << initial_state);
  ROS_INFO_STREAM("Final state:\t" << final_state);

  Eigen::VectorXd opt_abcd = EigenSolveQuadprog();
  spline_structure->SetCoefficients(opt_abcd);

  return spline_structure->GetPolynomials();
}

Eigen::VectorXd
QpFacade::EigenSolveQuadprog()
{
  Eigen::VectorXd opt_spline_coeff_xy;

  ROS_INFO_STREAM("n = " << cost_function_.M.rows() << " variables");
  ROS_INFO_STREAM("m_eq = " << equality_constraints_.M.rows() << " equality constraints");
  ROS_INFO_STREAM("m_in = " << inequality_constraints_.M.rows() << " inequality constraints");

  ROS_INFO("QP optimizer running...");

  clock_t start = std::clock();

  double cost = Eigen::solve_quadprog(
      cost_function_.M, cost_function_.v,
      equality_constraints_.M.transpose(), equality_constraints_.v,
      inequality_constraints_.M.transpose(), inequality_constraints_.v,
      opt_spline_coeff_xy);

  clock_t end = std::clock();
  double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;

  if (cost == std::numeric_limits<double>::infinity())
    throw std::length_error("Eigen::quadprog did not find a solution");
  else
    ROS_INFO_STREAM("QP optimizer solved in " << time << " ms.");

  std::cout << "cost: " << cost << std::endl;
  std::cout << "x: \n" << opt_spline_coeff_xy.transpose() << std::endl;

  return opt_spline_coeff_xy;
}


} // namespace zmp
} // namespace xpp
