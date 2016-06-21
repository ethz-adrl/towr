/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#include <xpp/zmp/qp_facade.h>

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/support_polygon_container.h>

#include <xpp/zmp/a_linear_constraint.h>
#include <xpp/zmp/initial_acceleration_equation.h>
#include <xpp/zmp/final_state_equation.h>
#include <xpp/zmp/spline_junction_equation.h>

#include <xpp/zmp/zmp_constraint_builder.h>

#include <xpp/zmp/total_acceleration_equation.h>
#include <xpp/utils/eigen_quadprog-inl.h>

#include <cmath>      // std::numeric_limits
#include <ros/console.h>

namespace xpp {
namespace zmp {

QpFacade::VecSpline
QpFacade::SolveQp(const State& initial_state,
                  const State& final_state,
                  const VecFoothold& start_stance,
                  const VecFoothold& steps,
                  const SplineTimes& times,
                  bool start_with_com_shift,
                  double robot_height)
{
  ContinuousSplineContainer spline_structure;
  spline_structure.Init(initial_state.p, initial_state.v , steps.size(), times,
                        start_with_com_shift, true);


  TotalAccelerationEquation total_acc_eq(spline_structure);
  cost_function_ = total_acc_eq.BuildLinearEquation();


  InitialAccelerationEquation eq_acc(initial_state.a, spline_structure.GetTotalFreeCoeff());
  FinalStateEquation eq_final(final_state, spline_structure);
  SplineJunctionEquation eq_junction(spline_structure);

  std::vector<ILinearEquationBuilder*> eq;
  eq.push_back(&eq_acc);
  eq.push_back(&eq_final);
  eq.push_back(&eq_junction);

  equality_constraints_ = MatVec(); // clear
  for (const ILinearEquationBuilder* const p : eq)
    equality_constraints_ << p->BuildLinearEquation();

  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  supp_polygon_container.Init(start_stance, steps, hyq::SupportPolygon::GetDefaultMargins());

  ROS_INFO_STREAM("Start polygon:\n" << supp_polygon_container.GetStartPolygon());
  std::vector<hyq::SupportPolygon> supp = supp_polygon_container.GetSupportPolygons();

  ROS_INFO_STREAM("Support polygons for the " << supp.size() << " steps:\n");
  for (const hyq::SupportPolygon& s : supp) {
    ROS_INFO_STREAM(s);
  }

  ZmpConstraintBuilder zmp_constraint(spline_structure, robot_height);
  inequality_constraints_ = zmp_constraint.CalcZmpConstraints(supp_polygon_container);

  ROS_INFO_STREAM("Initial state:\t" << initial_state);
  ROS_INFO_STREAM("Final state:\t" << final_state);

  Eigen::VectorXd opt_abcd = EigenSolveQuadprog();
  spline_structure.AddOptimizedCoefficients(opt_abcd);

  return spline_structure.GetSplines();
}

Eigen::VectorXd
QpFacade::EigenSolveQuadprog()
{
  Eigen::VectorXd opt_spline_coeff_xy;
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

  return opt_spline_coeff_xy;
}


} // namespace zmp
} // namespace xpp
