/*
 * continuous_spline_container.cc
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {


ContinuousSplineContainer::ContinuousSplineContainer (
    const Vector2d& start_cog_p,
    const Vector2d& start_cog_v,
    const std::vector<xpp::hyq::LegID>& step_sequence,
    double t_stance,
    double t_swing,
    double t_stance_initial,
    double t_stance_final)
    :SplineContainer(step_sequence, t_stance, t_swing, t_stance_initial, t_stance_final)
{
  Init(start_cog_p, start_cog_v, step_sequence, t_stance, t_swing, t_stance_initial, t_stance_final);
}


void ContinuousSplineContainer::Init(const Vector2d& start_cog_p,
                                     const Vector2d& start_cog_v,
                                     const std::vector<xpp::hyq::LegID>& step_sequence,
                                     double t_stance,
                                     double t_swing,
                                     double t_stance_initial,
                                     double t_stance_final)
{
  SplineContainer::Init(step_sequence, t_stance, t_swing, t_stance_initial, t_stance_final);

  for (int dim = xpp::utils::X; dim<=xpp::utils::Y; ++dim) {
    relationship_e_to_abcd_.at(dim) = DescribeEByABCD(dim, start_cog_v(dim));
    relationship_f_to_abdc_.at(dim) = DescribeFByABCD(dim, start_cog_p(dim), start_cog_v(dim));
  }
}


ContinuousSplineContainer::VecScalar
ContinuousSplineContainer::ExpressCogPosThroughABCD(double t_local, int id, int dim) const
{
  VecScalar pos(GetTotalFreeCoeff());

  VecScalar Ek = GetCoefficient(id, dim, E);
  VecScalar Fk = GetCoefficient(id, dim, F);

  // x_pos = at^5 +   bt^4 +  ct^3 + dt*2 + et + f
  pos.v(Index(id,dim,A))   = std::pow(t_local,5);
  pos.v(Index(id,dim,B))   = std::pow(t_local,4);
  pos.v(Index(id,dim,C))   = std::pow(t_local,3);
  pos.v(Index(id,dim,D))   = std::pow(t_local,2);
  pos.v                   += t_local*Ek.v;
  pos.v                   += 1*Fk.v;

  pos.s = Ek.s*t_local + Fk.s;

  return pos;
}


ContinuousSplineContainer::VecScalar
ContinuousSplineContainer::ExpressCogAccThroughABCD(double t_local, int id, int dim) const
{
  VecScalar acc(GetTotalFreeCoeff());

  int idx = Index(id,dim,A);
  acc.v.middleCols<kFreeCoeffPerSpline>(idx) = ExpressCogAccThroughABCD(t_local);

  return acc;
}


ContinuousSplineContainer::VecABCD
ContinuousSplineContainer::ExpressCogAccThroughABCD(double t_local) const
{
  VecABCD acc;

  // x_acc = 20at^3 + 12bt^2 + 6ct   + 2d
  acc(A)   = 20.0 * std::pow(t_local,3);
  acc(B)   = 12.0 * std::pow(t_local,2);
  acc(C)   =  6.0 * t_local;
  acc(D)   =  2.0;

  return acc;
}


int ContinuousSplineContainer::GetTotalFreeCoeff() const
{
  CheckIfSplinesInitialized();
  return splines_.size() * kFreeCoeffPerSpline * kDim2d;
}


int ContinuousSplineContainer::Index(int spline, int dim, int coeff)
{
  return kFreeCoeffPerSpline * kDim2d * spline + kFreeCoeffPerSpline * dim + coeff;
}


ContinuousSplineContainer::VecScalar
ContinuousSplineContainer::GetCoefficient(int spline_id_k, int dim, SplineCoeff c) const
{
  CheckIfSplinesInitialized();
  assert(c== E || c== F);
  const MatVec& rel = (c==E)? relationship_e_to_abcd_.at(dim) : relationship_f_to_abdc_.at(dim);
  return rel.GetRow(spline_id_k);
}


ContinuousSplineContainer::MatVec
ContinuousSplineContainer::DescribeEByABCD(int dim, double start_cog_v) const
{
  MatVec e_coeff(splines_.size(), GetTotalFreeCoeff());
  e_coeff.v[0] = start_cog_v;

  for (uint k=1; k<splines_.size(); ++k)
  {
    int kprev = k-1;

    // velocity at beginning of previous spline (e_prev)
    e_coeff.M.row(k) = e_coeff.M.row(kprev);
    e_coeff.v[k]     = e_coeff.v[kprev];

    // velocity change over previous spline due to a,b,c,d
    double Tkprev = splines_.at(kprev).GetDuration();
    e_coeff.M(k, Index(kprev,dim,A)) += 5*std::pow(Tkprev,4);
    e_coeff.M(k, Index(kprev,dim,B)) += 4*std::pow(Tkprev,3);
    e_coeff.M(k, Index(kprev,dim,C)) += 3*std::pow(Tkprev,2);
    e_coeff.M(k, Index(kprev,dim,D)) += 2*std::pow(Tkprev,1);
  }

  return e_coeff;
}


ContinuousSplineContainer::MatVec
ContinuousSplineContainer::DescribeFByABCD(int dim, double start_cog_p,
                                           double start_cog_v) const
{
  MatVec e_coeff = DescribeEByABCD(dim, start_cog_v);

  MatVec f_coeff(splines_.size(), GetTotalFreeCoeff());
  f_coeff.v[0] = start_cog_p;

  for (uint k=1; k<splines_.size(); ++k)
  {
    int kprev = k-1;
    // position at start of previous spline (=f_prev)
    f_coeff.M.row(k) = f_coeff.M.row(kprev);
    f_coeff.v[k]     = f_coeff.v[kprev];

    double Tkprev    = splines_.at(kprev).GetDuration();

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
    const Eigen::VectorXd& optimized_coeff,
    VecSpline& splines) const
{
  CheckIfSplinesInitialized();
  assert(splines.size() == (optimized_coeff.rows()/kDim2d/kFreeCoeffPerSpline));

  for (size_t k=0; k<splines.size(); ++k) {
    CoeffValues coeff_values;

    for (int dim=xpp::utils::X; dim<=xpp::utils::Y; dim++) {

      double* cv = (dim == xpp::utils::X) ? coeff_values.x : coeff_values.y;

      // fill in only first 4 optimized coefficients
      cv[A] = optimized_coeff[Index(k,dim,A)];
      cv[B] = optimized_coeff[Index(k,dim,B)];
      cv[C] = optimized_coeff[Index(k,dim,C)];
      cv[D] = optimized_coeff[Index(k,dim,D)];

      // calculate e and f coefficients from previous values
      VecScalar Ek = GetCoefficient(k, dim, E);
      VecScalar Fk = GetCoefficient(k, dim, F);

      cv[E] = Ek.v*optimized_coeff + Ek.s;
      cv[F] = Fk.v*optimized_coeff + Fk.s;

    } // dim:X..Y

    splines.at(k).SetSplineCoefficients(coeff_values);

  } // k=0..n_spline_infos_
}


} /* namespace zmp */
} /* namespace xpp */
