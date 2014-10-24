/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#ifndef ZMP_OPTIMIZER_H_
#define ZMP_OPTIMIZER_H_

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


struct SplineInfo {
  unsigned int id_;
  double duration_; // time during which this spline is active
  bool four_leg_supp_;
  int step_;

  SplineInfo(unsigned int id, double duration,
             bool four_leg_supp, int step)
      : id_(id), duration_(duration),
        four_leg_supp_(four_leg_supp), step_(step) {}
};

struct MatVec {
  Eigen::MatrixXd M;
  Eigen::VectorXd v;
  MatVec(int rows_m, int cols_m, int cols_v) {
    M = Eigen::MatrixXd::Zero(rows_m, cols_m);
    v = Eigen::VectorXd::Zero(cols_v);
  }
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
  typedef ::xpp::utils::Vec3d Position;
  typedef std::vector<Foothold> Footholds;
  typedef std::vector<SuppTriangle> SuppTriangles;
  typedef std::array<double,2> WeightsXYArray;
  typedef std::shared_ptr<MatVec> MatVecPtr;
  typedef std::vector<SplineInfo> SplineInfoVec;
  typedef std::vector<ZmpSpline> Splines;


public:
  ZmpOptimizer();
  /*!
  @brief C'tor defining important params for optimization.
  @param dt discretization interval for inequality constraints
  @param stability_margin: reduce support triangles by this distance [m]
  @param n_splines_per_step how many splines one step should be comprised of
  @param t_four_leg_support time [s] of four leg support phase when switching
         between disjoint support triangles
  */
  ZmpOptimizer(double dt, int n_splines_per_step, int n_splines_per_4ls,
               double t_swing, double t_four_leg_support, double e_and_f_cost);
  virtual ~ZmpOptimizer();

/*!
 @brief Optimizes the trajectory of the CoM according to a quadratic
        cost function while keeping the ZMP in the current support
        triangle formed by the stance feet.

 @param start_cog Current position of the robots center of gravity
 @param goal_cog Cog of robot after executing the steps
 @param start_stance Position of the four feet before walking
 @param steps Footholds the robot must execute
 @param[out] splines optimized cog trajectory
 @throws std::runtime_error Throws if no solution to the QP problem was found.
 */
  void OptimizeSplineCoeff(Position& start_cog,
                           const hyq::LegDataMap<Foothold>& start_stance, Footholds &steps,
                           const WeightsXYArray& weight, hyq::MarginValues margins,
                           double height_robot,
                           Splines& splines);


private:
  const double kDt; ///< discretization interval
  const int kSplinesPerStep;
  const int kSplinesPer4ls;
  const double kTimeSwing_;
  const double kTime4ls; ///< time four legged support phase
  double EandFCost;


  const SplineInfoVec ConstructSplineSequence(const std::vector<LegID>& step_sequence);
  MatVecPtr CreateMinAccCostFunction(const SplineInfoVec& s, const WeightsXYArray& weight);
  MatVecPtr CreateEqualityContraints(const SplineInfoVec& s, const Position &start_cog, const Position &end_cog) const;
  MatVecPtr CreateInequalityContraints(const SplineInfoVec& s, const hyq::SuppTriangles &tr,
                                       double height_robot) const;

  Splines CreateSplines(Eigen::VectorXd& coeff,
                        const SplineInfoVec& spline_infos) const;

  int var_index(int splines, int dim, int spline_coeff) const;

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

#endif /* ZMP_OPTIMIZER_H_ */
