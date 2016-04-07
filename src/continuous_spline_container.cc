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


void ContinuousSplineContainer::Init(const Eigen::Vector2d& start_cog_p,
                                     const Eigen::Vector2d& start_cog_v,
                                     const std::vector<xpp::hyq::LegID>& step_sequence,
                                     double t_stance,
                                     double t_swing,
                                     double t_stance_initial,
                                     double t_stance_final,
                                     double dt)
{
  ConstructSplineSequence(step_sequence, t_stance, t_swing, t_stance_initial, t_stance_final);
  dt_ = dt;
  initialized_ = true;

  for (int dim = xpp::utils::X; dim<=xpp::utils::Y; ++dim) {
    e_coefficients_.at(dim) = DescribeEByPrev(dim, start_cog_v(dim));
    f_coefficients_.at(dim) = DescribeFByPrev(dim, start_cog_p(dim), start_cog_v(dim));
  }
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


int ContinuousSplineContainer::GetTotalNodesNo4ls() const
{
  int node_count = 0;

  for (ZmpSpline s: splines_) {
    if (s.four_leg_supp_)
      continue;
    else
      node_count += s.GetNodeCount(dt_);
  };
  return node_count;
}


int ContinuousSplineContainer::GetTotalNodes4ls() const
{
  int node_count = 0;

  for (ZmpSpline s: splines_) {
    if (s.four_leg_supp_)
      node_count += s.GetNodeCount(dt_);
  };
  return node_count;
}


ContinuousSplineContainer::VecScalar
ContinuousSplineContainer::GetCalculatedCoeff(int spline_id_k, int dim, SplineCoeff c) const
{
  assert(c== E || c== F);
  const MatVec& coeff = (c==E)? e_coefficients_.at(dim) : f_coefficients_.at(dim);
  return coeff.ExtractRow(spline_id_k);
}


ContinuousSplineContainer::MatVec
ContinuousSplineContainer::DescribeEByPrev(int dim, double start_cog_v) const
{
  CheckIfInitialized();
  MatVec e_coeff(splines_.size(), GetTotalFreeCoeff());
  e_coeff.v[0] = start_cog_v;

  for (uint k=1; k<splines_.size(); ++k)
  {
    int kprev = k-1;

    // velocity at beginning of previous spline (e_prev)
    e_coeff.M.row(k) = e_coeff.M.row(kprev);
    e_coeff.v[k]     = e_coeff.v[kprev];

    // velocity change over previous spline due to a,b,c,d
    double Tkprev = splines_.at(kprev).duration_;
    e_coeff.M(k, Index(kprev,dim,A)) += 5*std::pow(Tkprev,4);
    e_coeff.M(k, Index(kprev,dim,B)) += 4*std::pow(Tkprev,3);
    e_coeff.M(k, Index(kprev,dim,C)) += 3*std::pow(Tkprev,2);
    e_coeff.M(k, Index(kprev,dim,D)) += 2*std::pow(Tkprev,1);
  }

  return e_coeff;
}


ContinuousSplineContainer::MatVec
ContinuousSplineContainer::DescribeFByPrev(int dim, double start_cog_p,
                                           double start_cog_v) const
{
  CheckIfInitialized();
  MatVec e_coeff = DescribeEByPrev(dim, start_cog_v);

  MatVec f_coeff(splines_.size(), GetTotalFreeCoeff());
  f_coeff.v[0] = start_cog_p;

  for (uint k=1; k<splines_.size(); ++k)
  {
    int kprev = k-1;
    // position at start of previous spline (=f_prev)
    f_coeff.M.row(k) = f_coeff.M.row(kprev);
    f_coeff.v[k]     = f_coeff.v[kprev];

    double Tkprev    = splines_.at(kprev).duration_;

    // position change over previous spline due to a,b,c,d
    f_coeff.M(k, Index(kprev,dim,A))        += std::pow(Tkprev,5);
    f_coeff.M(k, Index(kprev,dim,B))        += std::pow(Tkprev,4);
    f_coeff.M(k, Index(kprev,dim,C))        += std::pow(Tkprev,3);
    f_coeff.M(k, Index(kprev,dim,D))        += std::pow(Tkprev,2);
    // position change over previous spline due to e
    f_coeff.M.row(k) += e_coeff.M.row(kprev)  *std::pow(Tkprev,1);
    f_coeff.v[k]     += e_coeff.v[kprev]      *std::pow(Tkprev,1);
  }

  return f_coeff;
}


void
ContinuousSplineContainer::AddOptimizedCoefficients(
    const Eigen::VectorXd& optimized_coeff)
{
  CheckIfInitialized();

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
      VecScalar Ek = GetCalculatedCoeff(k, dim, E);
      VecScalar Fk = GetCalculatedCoeff(k, dim, F);

      cv[E] = Ek.v*optimized_coeff + Ek.s;
      cv[F] = Fk.v*optimized_coeff + Fk.s;

    } // dim:X..Y

    splines_.at(k).set_spline_coeff(coeff_values);

  } // k=0..n_spline_infos_
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
