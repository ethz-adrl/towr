/*
 * continuous_spline_container.cc
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/continuous_spline_container.h>

#include <ctime>      // std::clock_t
#include <cmath>      // std::numeric_limits

namespace xpp {
namespace zmp {

ContinuousSplineContainer::ContinuousSplineContainer ()
{
  // TODO Auto-generated constructor stub

}

ContinuousSplineContainer::~ContinuousSplineContainer ()
{
  // TODO Auto-generated destructor stub
}


void ContinuousSplineContainer::Init(const Eigen::Vector2d& start_cog_p,
                                     const Eigen::Vector2d& start_cog_v,
                                     const std::vector<xpp::hyq::LegID>& step_sequence,
                                     double t_stance,
                                     double t_swing,
                                     double t_stance_initial,
                                     double t_stance_final,
                                     double dt)
{
  SetInitialPosVel(start_cog_p, start_cog_v);
  ConstructSplineSequence(step_sequence, t_stance, t_swing, t_stance_initial, t_stance_final);
  dt_ = dt;
  initialized_ = true;
}

void ContinuousSplineContainer::SetInitialPosVel(const Eigen::Vector2d& start_cog_p,
                                                 const Eigen::Vector2d& start_cog_v)
{
  start_cog_p_ = start_cog_p;
  start_cog_v_ = start_cog_v;
}


int ContinuousSplineContainer::GetTotalFreeCoeff() const
{
  CheckIfInitialized();
  return splines_.size() * kFreeCoeffPerSpline * kDim2d;
}


int ContinuousSplineContainer::Index(int spline, int dim, int coeff)
{
  return kFreeCoeffPerSpline * kDim2d * spline + kFreeCoeffPerSpline * dim + coeff;
}


int ContinuousSplineContainer::GetTotalNodes(bool exclude_4ls_splines) const
{
  int node_count = 0;

  for (ZmpSpline s: splines_) {

    if (s.four_leg_supp_ && exclude_4ls_splines)
      continue;

    node_count += s.GetNodeCount(dt_);
  };
  return node_count;
}


// TODO create lookup table for this
int ContinuousSplineContainer::GetSplineID(int node) const
{
  int n = 0;
  for (ZmpSpline s: splines_) {
    n += s.GetNodeCount(dt_);

    if (n > node)
      return s.id_;
  }
  return splines_.back().id_;
}


void ContinuousSplineContainer::DescribeEByPrev(int k,
                                   int dim, Eigen::RowVectorXd& Ek,
                                   double& non_dependent) const
{
  CheckIfInitialized();

  Ek.setZero();
  for (int i=0; i<k; ++i) {
    double Ti = splines_.at(i).duration_;

    Ek(Index(i,dim,A)) += 5*std::pow(Ti,4);
    Ek(Index(i,dim,B)) += 4*std::pow(Ti,3);
    Ek(Index(i,dim,C)) += 3*std::pow(Ti,2);
    Ek(Index(i,dim,D)) += 2*std::pow(Ti,1);
  }

  non_dependent = start_cog_v_(dim);
}


void ContinuousSplineContainer::DescribeFByPrev(int k, int dim,
                                   Eigen::RowVectorXd& Fk, double& non_dependent) const
{
  CheckIfInitialized();

  Eigen::VectorXd T0tok(k); T0tok.setZero();
  Fk.setZero();
  for (int i=0; i<k; ++i) {

    double Ti = splines_.at(i).duration_;

    T0tok(i) = Ti; // initial velocity acts over the entire time T of EVERY spline

    Fk[Index(i,dim,A)] += std::pow(Ti,5);
    Fk[Index(i,dim,B)] += std::pow(Ti,4);
    Fk[Index(i,dim,C)] += std::pow(Ti,3);
    Fk[Index(i,dim,D)] += std::pow(Ti,2);

    for (int j = 0; j<i; ++j) {
      double Tj = splines_.at(j).duration_;
      Fk[Index(j,dim,A)] += 5*std::pow(Tj,4) * Ti;
      Fk[Index(j,dim,B)] += 4*std::pow(Tj,3) * Ti;
      Fk[Index(j,dim,C)] += 3*std::pow(Tj,2) * Ti;
      Fk[Index(j,dim,D)] += 2*std::pow(Tj,1) * Ti;
    }
  }

  non_dependent = start_cog_v_(dim)*T0tok.sum() + start_cog_p_(dim);
}


void
ContinuousSplineContainer::AddOptimizedCoefficients(
    const Eigen::VectorXd& optimized_coeff)
{
  CheckIfInitialized();

  Eigen::RowVectorXd Ek(optimized_coeff.size());
  Eigen::RowVectorXd Fk(optimized_coeff.size());
  double non_dependent_e, non_dependent_f;

  for (size_t k=0; k<splines_.size(); ++k) {
    CoeffValues coeff_values;

    for (int dim=xpp::utils::X; dim<=xpp::utils::Y; dim++) {

      double* cv = (dim == xpp::utils::X) ? coeff_values.x : coeff_values.y;

      // fill in only first 4 optimized coefficients
      cv[A] = optimized_coeff[Index(k,dim,A)];
      cv[B] = optimized_coeff[Index(k,dim,B)];
      cv[C] = optimized_coeff[Index(k,dim,C)];
      cv[D] = optimized_coeff[Index(k,dim,D)];

      // calculate e and f coefficients from previous values
      Ek.setZero();
      Fk.setZero();
      DescribeEByPrev(k, dim, Ek, non_dependent_e);
      DescribeFByPrev(k, dim, Fk, non_dependent_f);

      cv[E] = Ek*optimized_coeff + non_dependent_e;
      cv[F] = Fk*optimized_coeff + non_dependent_f;

    } // dim:X..Y

    splines_.at(k).set_spline_coeff(coeff_values);

  } // k=0..n_spline_infos_
}


xpp::utils::MatVec
ContinuousSplineContainer::ExpressZmpThroughCoefficients(double h, int dim) const
{
//  std::clock_t start = std::clock();
  CheckIfInitialized();

  int coeff = GetTotalFreeCoeff();
  int num_nodes_with_4ls = GetTotalNodes();
  int num_nodes = num_nodes_with_4ls;

  MatVec ineq(num_nodes, coeff);

  const double g = 9.81; // gravity acceleration
  int n = 0; // node counter

  for (const ZmpSpline& s : splines_) {
//    LOG4CXX_TRACE(log_, "Calc inequality constaints of spline " << s.id_ << " of " << splines_.size() << ", duration=" << std::setprecision(3) << s.duration_ << ", step=" << s.step_);

    // calculate e and f coefficients from previous values
    const int k = s.id_;
    Eigen::RowVectorXd Ek(coeff); Ek.setZero();
    Eigen::RowVectorXd Fk(coeff); Fk.setZero();
    double non_dependent_e, non_dependent_f;
    DescribeEByPrev(k, dim, Ek, non_dependent_e);
    DescribeFByPrev(k, dim, Fk, non_dependent_f);

    for (double i=0; i < s.GetNodeCount(dt_); ++i) {

      double time = i*dt_;
      std::array<double,6> t = utils::cache_exponents<6>(time);

      //  x_zmp = x_pos - height/(g+z_acc) * x_acc
      //      with  x_pos = at^5 +   bt^4 +  ct^3 + dt*2 + et + f
      //            x_acc = 20at^3 + 12bt^2 + 6ct   + 2d
      double z_acc = 0.0; // TODO: calculate z_acc based on foothold height

      ineq.M(n, Index(k,dim,A)) = t[5]     - h/(g+z_acc) * 20.0 * t[3];
      ineq.M(n, Index(k,dim,B)) = t[4]     - h/(g+z_acc) * 12.0 * t[2];
      ineq.M(n, Index(k,dim,C)) = t[3]     - h/(g+z_acc) *  6.0 * t[1];
      ineq.M(n, Index(k,dim,D)) = t[2]     - h/(g+z_acc) *  2.0;
      ineq.M.row(n)            += t[1]*Ek;
      ineq.M.row(n)            += t[0]*Fk;

      ineq.v[n] = non_dependent_e*t[0] + non_dependent_f;

      ++n;
    }
  }


//  LOG4CXX_INFO(log_, "Calc. time inequality constraints:\t" << static_cast<double>(std::clock() - start) / CLOCKS_PER_SEC * 1000.0  << "\tms");
//  LOG4CXX_DEBUG(log_, "Dim: " << ineq.M.rows() << " x " << ineq.M.cols());
//  LOG4CXX_TRACE(log_, "Matrix:\n" << std::setprecision(2) << ineq.M << "\nVector:\n" << ineq.v.transpose());
  return ineq;
}


void
ContinuousSplineContainer::CheckIfInitialized() const
{
  if (!initialized_) {
    throw std::runtime_error("ContinousSplineContainer not initialized. Call Init() first");
  }
}

} /* namespace zmp */
} /* namespace xpp */
