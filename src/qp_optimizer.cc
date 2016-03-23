/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#include <xpp/zmp/qp_optimizer.h>

#include <xpp/zmp/eigen_quadprog-inl.h>
#include <xpp/hyq/supp_triangle_container.h>
#include <xpp/utils/logger_helpers-inl.h>


#include <ctime>      // std::clock_t
#include <cmath>      // std::numeric_limits
#include <algorithm>  // std::count

namespace xpp {
namespace zmp {

using hyq::LF; using hyq::RF; using hyq::LH; using hyq::RH;
using utils::X; using utils::Y; using utils::Z;

log4cxx::LoggerPtr  QpOptimizer::log_ = log4cxx::Logger::getLogger("xpp.zmp.zmpoptimizer");
log4cxx::LoggerPtr  QpOptimizer::log_matlab_ = log4cxx::Logger::getLogger("matlab");

QpOptimizer::QpOptimizer()
{
  LOG4CXX_WARN(log_, "default params are set!");
}

QpOptimizer::QpOptimizer(const ContinuousSplineContainer& spline_structure)
    :zmp_splines_(spline_structure)
{
}


QpOptimizer::~QpOptimizer() {}


void QpOptimizer::SetupQpMatrices(
    const WeightsXYArray& weight,
    const xpp::hyq::SuppTriangleContainer& supp_triangle_container,
    double walking_height)
{
  if (zmp_splines_.splines_.empty()) {
    throw std::runtime_error("zmp.optimizer.cc: spline_info vector empty. First call ConstructSplineSequence()");
  }

  cf_ = CreateMinAccCostFunction(weight);

  Position end_cog = supp_triangle_container.GetCenterOfFinalStance();

  eq_ = CreateEqualityContraints(end_cog);

  ineq_ = CreateInequalityContraints(supp_triangle_container, walking_height);
}


QpOptimizer::MatVec
QpOptimizer::CreateInequalityContraints(const xpp::hyq::SuppTriangleContainer& supp_triangle_container, double walking_height)
{
  MatVec zmp_x = zmp_splines_.ExpressZmpThroughCoefficients(walking_height, X);
  MatVec zmp_y = zmp_splines_.ExpressZmpThroughCoefficients(walking_height, Y);

  return supp_triangle_container.AddLineConstraints(zmp_x, zmp_y, zmp_splines_);
}


Eigen::VectorXd QpOptimizer::SolveQp() {

  Eigen::VectorXd opt_spline_coeff_xy;

  clock_t start = std::clock();

  double cost = Eigen::solve_quadprog(cf_.M, cf_.v, eq_.M, eq_.v, ineq_.M, ineq_.v,
                                      opt_spline_coeff_xy);
  clock_t end = std::clock();

  LOG4CXX_INFO(log_, "Time QP solver:\t\t" << static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0 << "\tms");
  LOG4CXX_INFO(log_, "Cost:\t\t" << cost);
  LOG4CXX_INFO(log_matlab_, opt_spline_coeff_xy.transpose());


  if (cost == std::numeric_limits<double>::infinity() || cost < 0.002)
    throw std::length_error("Eigen::quadprog did not find a solution");

  LOG4CXX_TRACE(log_, "x = " << opt_spline_coeff_xy.transpose()); //ax1, bx1, cx1, dx1, ex1, fx1 -- ay1, by1, cy1, dy1, ey1, fy1 -- ax2, bx2, cx2, dx2, ex2, fx2 -- ay2, by2, cy2, dy2, ey2, fy2 ...
  return opt_spline_coeff_xy;
}


QpOptimizer::MatVec
QpOptimizer::CreateMinAccCostFunction(const WeightsXYArray& weight) const
{
  std::clock_t start = std::clock();

  // total number of coefficients to be optimized
  int n_coeff = zmp_splines_.GetTotalFreeCoeff();
  MatVec cf(n_coeff, n_coeff);

  for (const ZmpSpline& s : zmp_splines_.splines_) {
    std::array<double,10> t_span = utils::cache_exponents<10>(s.duration_);

    for (int dim = X; dim <= Y; dim++) {
      const int a = ContinuousSplineContainer::Idx(s.id_, dim, A);
      const int b = ContinuousSplineContainer::Idx(s.id_, dim, B);
      const int c = ContinuousSplineContainer::Idx(s.id_, dim, C);
      const int d = ContinuousSplineContainer::Idx(s.id_, dim, D);

      // for explanation of values see M.Kalakrishnan et al., page 248
      // "Learning, Planning and Control for Quadruped Robots over challenging
      // Terrain", IJRR, 2010
      cf.M(a, a) = 400.0 / 7.0      * t_span[7] * weight[dim];
      cf.M(a, b) = 40.0             * t_span[6] * weight[dim];
      cf.M(a, c) = 120.0 / 5.0      * t_span[5] * weight[dim];
      cf.M(a, d) = 10.0             * t_span[4] * weight[dim];
      cf.M(b, b) = 144.0 / 5.0      * t_span[5] * weight[dim];
      cf.M(b, c) = 18.0             * t_span[4] * weight[dim];
      cf.M(b, d) = 8.0              * t_span[3] * weight[dim];
      cf.M(c, c) = 12.0             * t_span[3] * weight[dim];
      cf.M(c, d) = 6.0              * t_span[2] * weight[dim];
      cf.M(d, d) = 4.0              * t_span[1] * weight[dim];

      // mirrow values over diagonal to fill bottom left triangle
      for (int c = 0; c < cf.M.cols(); ++c)
        for (int r = c + 1; r < cf.M.rows(); ++r)
          cf.M(r, c) = cf.M(c, r);
    }
  }

  LOG4CXX_INFO(log_, "Calc. time cost function:\t\t" << static_cast<double>(std::clock() - start) / CLOCKS_PER_SEC * 1000.0  << "\tms");
  LOG4CXX_TRACE(log_, "Matrix:\n" << std::setprecision(2) << cf.M << "\nVector:\n" << cf.v.transpose());
  return cf;
}

QpOptimizer::MatVec
QpOptimizer::CreateEqualityContraints(const Position &end_cog) const
{
  std::clock_t start = std::clock();

  int coeff = zmp_splines_.GetTotalFreeCoeff();; // total number of all spline coefficients
  int constraints = 0;
  constraints += kDim2d*2;                         // init {x,y} * {acc, jerk} pos, vel implied set through spline_container.AddOptimizedCoefficients()
  constraints += kDim2d*3;                         // end  {x,y} * {pos, vel, acc}
  constraints += (zmp_splines_.splines_.size()-1) * kDim2d * 2;  // junctions {acc,jerk} since pos, vel  implied
  MatVec ec(coeff, constraints);

  const Eigen::Vector2d kAccStart = Eigen::Vector2d(0.0, 0.0);
  const Eigen::Vector2d kJerkStart= Eigen::Vector2d(0.0, 0.0);
  const Eigen::Vector2d kVelEnd   = Eigen::Vector2d(0.0, 0.0);
  const Eigen::Vector2d kAccEnd   = Eigen::Vector2d(0.0, 0.0);


  int i = 0; // counter of equality constraints
  for (int dim = X; dim <= Y; ++dim)
  {
    // 2. Initial conditions
    // acceleration set to zero
    int d = ContinuousSplineContainer::Idx(0, dim, D);
    ec.M(d, i) = 2.0;
    ec.v(i++) = -kAccStart(dim);
    // jerk set to zero
    int c = ContinuousSplineContainer::Idx(0, dim, C);
    ec.M(c, i) = 6.0;
    ec.v(i++) = -kJerkStart(dim);


    // 2. Final conditions
    int K = zmp_splines_.splines_.back().id_; // id of last spline
    int last_spline = ContinuousSplineContainer::Idx(K, dim, A);
    std::array<double,6> t_duration = utils::cache_exponents<6>(zmp_splines_.splines_.back().duration_);

    // calculate e and f coefficients from previous values
    Eigen::VectorXd Ek(coeff); Ek.setZero();
    Eigen::VectorXd Fk(coeff); Fk.setZero();
    double non_dependent_e, non_dependent_f;
    zmp_splines_.DescribeEByPrev(K, dim, Ek, non_dependent_e);
    zmp_splines_.DescribeFByPrev(K, dim, Fk, non_dependent_f);

    // position
    ec.M(last_spline + A, i) = t_duration[5];
    ec.M(last_spline + B, i) = t_duration[4];
    ec.M(last_spline + C, i) = t_duration[3];
    ec.M(last_spline + D, i) = t_duration[2];
    ec.M.col(i) += Ek*t_duration[1];
    ec.M.col(i) += Fk;

    ec.v(i)     += non_dependent_e*t_duration[1] + non_dependent_f;
    ec.v(i++)   += -end_cog(dim);

    // velocities
    ec.M(last_spline + A, i) = 5 * t_duration[4];
    ec.M(last_spline + B, i) = 4 * t_duration[3];
    ec.M(last_spline + C, i) = 3 * t_duration[2];
    ec.M(last_spline + D, i) = 2 * t_duration[1];
    ec.M.col(i) += Ek;

    ec.v(i)     += non_dependent_e;
    ec.v(i++)   += -kVelEnd(dim);

    // accelerations
    ec.M(last_spline + A, i) = 20 * t_duration[3];
    ec.M(last_spline + B, i) = 12 * t_duration[2];
    ec.M(last_spline + C, i) = 6  * t_duration[1];
    ec.M(last_spline + D, i) = 2;

    ec.v(i++) = -kAccEnd(dim);
  }

  // 3. Equal conditions at spline junctions
  for (uint s = 0; s < zmp_splines_.splines_.size() - 1; ++s)
  {
    std::array<double,6> t_duration = utils::cache_exponents<6>(zmp_splines_.splines_.at(s).duration_);
    for (int dim = X; dim <= Y; dim++) {

      int curr_spline = ContinuousSplineContainer::Idx(s, dim, A);
      int next_spline = ContinuousSplineContainer::Idx(s + 1, dim, A);

      // acceleration
      ec.M(curr_spline + A, i) = 20 * t_duration[3];
      ec.M(curr_spline + B, i) = 12 * t_duration[2];
      ec.M(curr_spline + C, i) = 6  * t_duration[1];
      ec.M(curr_spline + D, i) = 2;
      ec.M(next_spline + D, i) = -2.0;
      ec.v(i++) = 0.0;

      // jerk (derivative of acceleration)
      ec.M(curr_spline + A, i) = 60 * t_duration[2];
      ec.M(curr_spline + B, i) = 24 * t_duration[1];
      ec.M(curr_spline + C, i) = 6;
      ec.M(next_spline + C, i) = -6.0;
      ec.v(i++) = 0.0;
    }
  }

  LOG4CXX_INFO(log_, "Calc. time equality constraints:\t" << static_cast<double>(std::clock() - start) / CLOCKS_PER_SEC * 1000.0  << "\tms");
  LOG4CXX_DEBUG(log_, "Dim: " << ec.M.rows() << " x " << ec.M.cols());
  LOG4CXX_TRACE(log_, "Matrix:\n" << std::setprecision(2) << ec.M << "\nVector:\n" << ec.v.transpose());
  return ec;
}


} // namespace zmp
} // namespace xpp
