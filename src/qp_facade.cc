/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#include <xpp/zmp/qp_facade.h>

#include <xpp/zmp/motion_factory.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/linear_spline_equations.h>
#include <xpp/zmp/zmp_constraint_builder.h>
#include <xpp/utils/eigen_quadprog-inl.h>

#include <cmath>      // std::numeric_limits
#include <ctime>      // std::clock()

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
  // remember to comment in regularization when using this type of spline representation
//  auto com_spline = MotionFactory::CreateComMotion(initial_state.p, initial_state.v , steps.size(), times,start_with_com_shift);
  auto com_spline = MotionFactory::CreateComMotion(steps.size(), times, start_with_com_shift);

  LinearSplineEquations spline_eq(com_spline);

  cost_function_ = spline_eq.MakeAcceleration(1.0,3.0);

  // comment this in if using ComSpline6
  // because the matrix has to be positive definite for eigen quadprog
  Eigen::MatrixXd reg = cost_function_.M;
  reg.setIdentity();
  cost_function_.M += 1e-10*reg;


  equality_constraints_ = MatVec(); // clear
  equality_constraints_ << spline_eq.MakeInitial(initial_state);
  equality_constraints_ << spline_eq.MakeFinal(final_state);
  equality_constraints_ << spline_eq.MakeJunction();

  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  supp_polygon_container.Init(start_stance, steps, hyq::SupportPolygon::GetDefaultMargins());

//  std::cout << "Start polygon:\n" << supp_polygon_container.GetStartPolygon());
  std::vector<hyq::SupportPolygon> supp = supp_polygon_container.GetSupportPolygons();

  std::cout << "number of support polygons: " << supp.size() << std::endl;
//  for (const hyq::SupportPolygon& s : supp) std::cout << s;

  ZmpConstraintBuilder zmp_constraint(com_spline, supp_polygon_container, robot_height);
  MatVecVec zmp_constr = zmp_constraint.CalcZmpConstraints(supp_polygon_container);
  inequality_constraints_.M = zmp_constr.Mv.M;
  inequality_constraints_.v = zmp_constr.Mv.v + zmp_constr.constant;

//  std::cout << "zmp_constr.Mv.v" << zmp_constr.Mv.v.transpose() << std::endl;
//  std::cout << "zmp_constr.constant" << zmp_constr.constant.transpose() << std::endl;

  std::cout << "Initial state:\t" << initial_state << std::endl;
  std::cout << "Final state:\t" << final_state << std::endl;

  Eigen::VectorXd opt_abcd = EigenSolveQuadprog();
  com_spline->SetCoefficients(opt_abcd);

  return com_spline->GetPolynomials();
}

Eigen::VectorXd
QpFacade::EigenSolveQuadprog()
{
  Eigen::VectorXd opt_spline_coeff_xy;

  std::cout << "n = " << cost_function_.M.rows() << " variables\n";
  std::cout << "m_eq = " << equality_constraints_.M.rows() << " equality constraints\n";
  std::cout << "m_in = " << inequality_constraints_.M.rows() << " inequality constraints\n";

  std::cout << "QP optimizer running...\n";

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
    std::cout << "QP optimizer solved in " << time << " ms.\n";

  std::cout << "cost: " << cost << std::endl;
  std::cout << "x: \n" << opt_spline_coeff_xy.transpose() << std::endl;

  return opt_spline_coeff_xy;
}


} // namespace zmp
} // namespace xpp
