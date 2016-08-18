/*
 * continuous_spline_container.cc
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/continuous_spline_container.h>
#include <Eigen/LU>

namespace xpp {
namespace zmp {

using namespace xpp::utils::coords_wrapper;

ContinuousSplineContainer::ContinuousSplineContainer (
    const Vector2d& start_cog_p,
    const Vector2d& start_cog_v,
    int step_count,
    const SplineTimes& times)
{
  Init(start_cog_p, start_cog_v, step_count, times);
}


void ContinuousSplineContainer::Init(const Vector2d& start_cog_p,
                                     const Vector2d& start_cog_v,
                                     int step_count,
                                     const SplineTimes& times,
                                     bool insert_initial_stance)
{
  splines_.clear();

  // build the spline structure
  if (insert_initial_stance) {
    const int n_stance_splines = 2; // 3 allows quicker reaction
    double t = times.t_stance_initial_/n_stance_splines;
    for (int i=0; i<n_stance_splines; ++i)
      SplineContainer::AddStanceSpline(t);

//    double t_reaction = 0.06;
//    SplineContainer::AddStanceSpline(t_reaction);
//    SplineContainer::AddStanceSpline(times.t_stance_initial_-t_reaction);
  }


  SplineContainer::AddSplinesStepSequence(step_count, times.t_swing_);

  for (const Coords3D dim : Coords2DArray) {
    relationship_e_to_abcd_.at(dim) = DescribeEByABCD(dim, start_cog_v(dim));
    relationship_f_to_abdc_.at(dim) = DescribeFByABCD(dim, start_cog_p(dim), start_cog_v(dim));
  }

  // initialize all other coefficients apart from e,f of first spline to zero
  Eigen::VectorXd abcd(GetTotalFreeCoeff());
  abcd.setZero();
  AddOptimizedCoefficients(abcd);
}

ContinuousSplineContainer::VecScalar
ContinuousSplineContainer::ExpressComThroughCoeff (
    xpp::utils::PosVelAcc posVelAcc, double t_local, int id, Coords dim) const
{
  switch (posVelAcc) {
    case xpp::utils::kPos:
      return ExpressCogPosThroughABCD(t_local, id, dim);
    case xpp::utils::kVel:
      return ExpressCogVelThroughABCD(t_local, id, dim);
    case xpp::utils::kAcc:
      return ExpressCogAccThroughABCD(t_local, id, dim);
  }
}


void
ContinuousSplineContainer::AddOptimizedCoefficients(const VectorXd& optimized_coeff)
{
  AddOptimizedCoefficients(optimized_coeff, splines_);
}

ContinuousSplineContainer::VecSpline
ContinuousSplineContainer::BuildOptimizedSplines(const VectorXd& optimized_coeff) const
{
  VecSpline splines = splines_;
  AddOptimizedCoefficients(optimized_coeff, splines);
  return splines;
}

ContinuousSplineContainer::VecScalar
ContinuousSplineContainer::ExpressCogPosThroughABCD(double t_local, int id, Coords dim) const
{
  VecScalar pos(GetTotalFreeCoeff());

  VecScalar Ek = GetECoefficient(id, dim);
  VecScalar Fk = GetFCoefficient(id, dim);

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
ContinuousSplineContainer::ExpressCogVelThroughABCD (double t_local, int id,
                                                     Coords dim) const
{
  VecScalar vel(GetTotalFreeCoeff());

  VecScalar Ek = GetECoefficient(id, dim);

  // x_vel = 5at^4 +   4bt^3 +  3ct^2 + 2dt + e
  vel.v(Index(id,dim,A))   = 5 * std::pow(t_local,4);
  vel.v(Index(id,dim,B))   = 4 * std::pow(t_local,3);
  vel.v(Index(id,dim,C))   = 3 * std::pow(t_local,2);
  vel.v(Index(id,dim,D))   = 2 * std::pow(t_local,1);
  vel.v                   += Ek.v;

  vel.s = Ek.s;

  return vel;
}


ContinuousSplineContainer::VecScalar
ContinuousSplineContainer::ExpressCogAccThroughABCD(double t_local, int id, Coords dim) const
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

ContinuousSplineContainer::VectorXd
ContinuousSplineContainer::GetABCDCoeffients () const
{
  VectorXd x_abcd(GetTotalFreeCoeff());

  for (const auto& s : splines_)
    for (auto dim : Coords2DArray)
      for (auto coeff : SplineCoeffArray)
        x_abcd[Index(s.GetId(), dim, coeff)] = s.GetCoefficient(dim, coeff);

  return x_abcd;
}

int ContinuousSplineContainer::Index(int spline, Coords dim, SplineCoeff coeff)
{
  return kFreeCoeffPerSpline * kDim2d * spline + kFreeCoeffPerSpline * dim + coeff;
}


//ContinuousSplineContainer::VecScalar
//ContinuousSplineContainer::GetCoefficient(int spline_id_k, Coords dim, SplineCoeff c) const
//{
//  CheckIfSplinesInitialized();
//  assert(c== E || c== F);
//  const MatVec& rel = (c==E)? relationship_e_to_abcd_.at(dim) : relationship_f_to_abdc_.at(dim);
//  return rel.GetRow(spline_id_k);
//}


ContinuousSplineContainer::VecScalar
ContinuousSplineContainer::GetECoefficient(int spline_id_k, Coords dim) const
{
  CheckIfSplinesInitialized();
  return relationship_e_to_abcd_.at(dim).GetRow(spline_id_k);
}


ContinuousSplineContainer::VecScalar
ContinuousSplineContainer::GetFCoefficient(int spline_id_k, Coords dim) const
{
  CheckIfSplinesInitialized();
  return relationship_f_to_abdc_.at(dim).GetRow(spline_id_k);
}



ContinuousSplineContainer::MatVec
ContinuousSplineContainer::DescribeEByABCD(Coords dim, double start_cog_v) const
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
ContinuousSplineContainer::DescribeFByABCD(Coords dim, double start_cog_p,
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

    for (const Coords3D dim : Coords2DArray) {

      double* cv = (dim == xpp::utils::X) ? coeff_values.x : coeff_values.y;

      // fill in only first 4 optimized coefficients
      cv[A] = optimized_coeff[Index(k,dim,A)];
      cv[B] = optimized_coeff[Index(k,dim,B)];
      cv[C] = optimized_coeff[Index(k,dim,C)];
      cv[D] = optimized_coeff[Index(k,dim,D)];

      // calculate e and f coefficients from previous values
      VecScalar Ek = GetECoefficient(k, dim);
      VecScalar Fk = GetFCoefficient(k, dim);

      cv[E] = Ek.v*optimized_coeff + Ek.s;
      cv[F] = Fk.v*optimized_coeff + Fk.s;

    } // dim:X..Y

    splines.at(k).SetSplineCoefficients(coeff_values);

  } // k=0..n_spline_infos_
}

void
ContinuousSplineContainer::SetEndAtStart ()
{
  const Vector2d& start_com_v = splines_.front().GetState(xpp::utils::kVel,0.0);
  double T = splines_.front().GetDuration();

  Eigen::Matrix2d A; A << 3*T*T, 2*T, // velocity equal to zero
                          T*T*T, T*T; // position equal to initial
  Vector2d b(1,T);

  Vector2d c_and_d_x = A.lu().solve(-start_com_v.x()*b);
  Vector2d c_and_d_y = A.lu().solve(-start_com_v.y()*b);

  Eigen::VectorXd abcd(GetTotalFreeCoeff());
  abcd.setZero();
  abcd[Index(0,X,C)] = c_and_d_x(0);
  abcd[Index(0,X,D)] = c_and_d_x(1);
  abcd[Index(0,Y,C)] = c_and_d_y(0);
  abcd[Index(0,Y,D)] = c_and_d_y(1);

  AddOptimizedCoefficients(abcd);
}

} /* namespace zmp */
} /* namespace xpp */
