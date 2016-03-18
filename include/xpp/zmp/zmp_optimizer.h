/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#ifndef _XPP_ZMP_OPTIMIZER_H_
#define _XPP_ZMP_OPTIMIZER_H_

#include "spline_container.h"
#include <xpp/hyq/supp_triangle.h>
#include <xpp/hyq/foothold.h>
#include <xpp/utils/geometric_structs.h>

#include <log4cxx/logger.h>
#include <Eigen/Dense>

#include <memory> //std::shared_ptr
#include <array>
#include <vector>


/**
@brief Dynamic ("pp" = ++) locomotion ("x"-coordinate)

All functions related to hyq, all utilities and all optimizations are a nested
namespace of this.
 */
namespace xpp {

/**
@brief Zero-Moment-Point (ZMP) optimization

All classes that optimize the CoG trajectory while keeping the ZMP inside the
current support triangle are located in this namespace
 */
namespace zmp {


struct MatVec {
  Eigen::MatrixXd M;
  Eigen::VectorXd v;
  MatVec(int rows, int cols) {
    M = Eigen::MatrixXd::Zero(rows, cols);
    v = Eigen::VectorXd::Zero(cols);
  }
  MatVec() {}
};


/**
\class ZmpOptimizer
\brief Optimizes spline coefficients w.r.t Zero-Moment-Point criteria.

This class provides and interface to optimize the trajectory of the CoM of HyQ
for a specific sequence of footsteps. The resulting trajectory will always keep
the ZMP inside the current support triangle. The QP-solver implemented in
eigen_quadprog.hpp performs the optimization.

Some stuff here and there
\snippet example.cc Adding a resource
 */
class ZmpOptimizer {
public:
  typedef ::xpp::hyq::SuppTriangle SuppTriangle;
  typedef ::xpp::hyq::Foothold Foothold;
  typedef ::xpp::hyq::LegID LegID;
  typedef ::xpp::utils::Vec2d Position;
  typedef ::xpp::utils::Vec2d Velocity;
  typedef std::vector<Foothold> Footholds;
  typedef std::vector<SuppTriangle> SuppTriangles;
  typedef std::array<double,2> WeightsXYArray;
  typedef SplineContainer Splines;
  typedef SplineContainer S;

public:
  ZmpOptimizer();

  /**
   * @param spline_structure the amount and sequence of splines with empty coefficients
   */
  ZmpOptimizer(const Splines& spline_structure);

  virtual ~ZmpOptimizer();

/*!
 @brief Optimizes the trajectory of the CoM according to a quadratic
        cost function while keeping the ZMP in the current support
        triangle formed by the stance feet.

 @param start_cog Current position of the robots center of gravity
 @param goal_cog Cog of robot after executing the steps
 @param start_stance Position of the four feet before walking
 @param t_stance_initial The time permitted for the first 4-leg-support cog shift
 @param steps Footholds the robot must execute
 @param[out] splines optimized cog trajectory
 @throws std::runtime_error Throws if no solution to the QP problem was found.
 */
  void SetupQpMatrices(const Position& start_cog_p,
                           const Velocity& start_cog_v,
                           const hyq::LegDataMap<Foothold>& start_stance,
                           const Footholds &steps,
                           const WeightsXYArray& weight, hyq::MarginValues margins,
                           double height_robot);

  Eigen::VectorXd SolveQp();

  Eigen::VectorXd SolveIpopt(Eigen::VectorXd& final_footholds,
                             const Eigen::VectorXd& opt_coefficients_eig = Eigen::Vector2d::Zero());



  MatVec cf_;
  MatVec eq_;
  MatVec ineq_;

  Eigen::MatrixXd ineq_ipopt_;
  Eigen::VectorXd ineq_ipopt_vx_;
  Eigen::VectorXd ineq_ipopt_vy_;

  std::vector<SuppTriangle::TrLine> lines_for_constraint_;
  Footholds footholds_;

  std::vector<SuppTriangle::TrLine>
  LineForConstraint(const SuppTriangles &supp_triangles, double dt);

  MatVec CreateInequalityContraints(const Position& start_cog_p,
                                       const Velocity& start_cog_v,
                                       const std::vector<SuppTriangle::TrLine> &line_for_constraint,
                                       double height_robot,
                                       double dt);
private:
  Splines zmp_splines_;


  MatVec CreateMinAccCostFunction(const WeightsXYArray& weight) const;
  MatVec CreateEqualityContraints(const Position &start_cog_p,
                                     const Velocity &start_cog_v,
                                     const Position &end_cog) const;

  template<std::size_t N>
  std::array<double,N> cache_exponents(double t) const;

  static log4cxx::LoggerPtr log_;
  static log4cxx::LoggerPtr log_matlab_;
};

template<std::size_t N>
std::array<double,N> ZmpOptimizer::cache_exponents(double t) const
{
  std::array<double,N> exp = {{ 1.0, t }};
  for (uint e = 2; e < N; ++e)
    exp[e] = exp[e-1] * t;
  return exp;
}


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_OPTIMIZER_H_
