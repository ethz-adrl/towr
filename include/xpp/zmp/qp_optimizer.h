/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#ifndef _XPP_ZMP_OPTIMIZER_H_
#define _XPP_ZMP_OPTIMIZER_H_

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/supp_triangle_container.h>
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

 */
class QpOptimizer {
public:
  typedef ::xpp::hyq::SuppTriangle SuppTriangle;
  typedef ::xpp::utils::Vec2d Position;
  typedef std::array<double,2> WeightsXYArray;
  typedef ContinuousSplineContainer S;

public:
  QpOptimizer();

  /**
   * @param spline_structure the amount and sequence of splines with empty coefficients
   */
  QpOptimizer(const S& spline_structure);

  virtual ~QpOptimizer();

/*!
 @brief Optimizes the trajectory of the CoM according to a quadratic
        cost function while keeping the ZMP in the current support
        triangle formed by the stance feet.

 @param start_stance Position of the four feet before walking
 @param t_stance_initial The time permitted for the first 4-leg-support cog shift
 @param steps Footholds the robot must execute
 @param[out] splines optimized cog trajectory
 @throws std::runtime_error Throws if no solution to the QP problem was found.
 */
  void SetupQpMatrices(const WeightsXYArray& weight,
                       const xpp::hyq::SuppTriangleContainer& supp_triangle_container,
                       double height_robot);

  Eigen::VectorXd SolveQp();


  MatVec cf_;
  MatVec eq_;
  MatVec ineq_;

  // rename to get zmp
  MatVec CreateInequalityContraints(const std::vector<SuppTriangle> &supp_triangles,
                                    double walking_height);
  MatVec ExpressZmpThroughCoefficients(double walking_height, int dim) const;
  MatVec AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                            const std::vector<SuppTriangle> &supp_triangles) const;
  void AddLineConstraint(const SuppTriangle::TrLine& l,
                         const Eigen::VectorXd& x_zmp_M,
                         const Eigen::VectorXd& y_zmp_M,
                         double x_zmp_v,
                         double y_zmp_v,
                         int& c, Eigen::MatrixXd& M, Eigen::VectorXd& v) const;
  S zmp_splines_;
private:


  MatVec CreateMinAccCostFunction(const WeightsXYArray& weight) const;
  MatVec CreateEqualityContraints(const Position &end_cog) const;

  template<std::size_t N>
  std::array<double,N> cache_exponents(double t) const;

  static log4cxx::LoggerPtr log_;
  static log4cxx::LoggerPtr log_matlab_;
};

template<std::size_t N>
std::array<double,N> QpOptimizer::cache_exponents(double t) const
{
  std::array<double,N> exp = {{ 1.0, t }};
  for (uint e = 2; e < N; ++e)
    exp[e] = exp[e-1] * t;
  return exp;
}


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_OPTIMIZER_H_
