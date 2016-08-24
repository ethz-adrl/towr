/**
@file    com_spline4.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   Defines ComSpline4, which realizes a ComSpline
 */

#include <xpp/zmp/com_spline4.h>
#include <Eigen/LU>

namespace xpp {
namespace zmp {

using namespace xpp::utils::coords_wrapper;

ComSpline4::ComSpline4 ()
{
  // TODO Auto-generated constructor stub
}

ComSpline4::~ComSpline4 ()
{
  // TODO Auto-generated destructor stub
}

ComSpline4::ComSpline4 (
    const Vector2d& start_cog_p,
    const Vector2d& start_cog_v,
    int step_count,
    const SplineTimes& times)
{
  Init(start_cog_p, start_cog_v, step_count, times);
}


void ComSpline4::Init(const Vector2d& start_cog_p,
                                     const Vector2d& start_cog_v,
                                     int step_count,
                                     const SplineTimes& times,
                                     bool insert_initial_stance)
{
  polynomials_.clear();

  // build the spline structure
  if (insert_initial_stance) {
    const int n_stance_splines = 2; // 3 allows quicker reaction
    double t = times.t_stance_initial_/n_stance_splines;
    for (int i=0; i<n_stance_splines; ++i)
      AddStanceSpline(t);

//    double t_reaction = 0.06;
//    SplineContainer::AddStanceSpline(t_reaction);
//    SplineContainer::AddStanceSpline(times.t_stance_initial_-t_reaction);
  }


  AddSplinesStepSequence(step_count, times.t_swing_);

  for (const Coords3D dim : Coords2DArray) {
    relationship_e_to_abcd_.at(dim) = DescribeEByABCD(dim, start_cog_v(dim));
    relationship_f_to_abdc_.at(dim) = DescribeFByABCD(dim, start_cog_p(dim), start_cog_v(dim));
  }

  // initialize all other coefficients apart from e,f of first spline to zero
  Eigen::VectorXd abcd(GetTotalFreeCoeff());
  abcd.setZero();
  SetCoefficients(abcd);
}


void
ComSpline4::SetCoefficients(const VectorXd& optimized_coeff)
{
  AddOptimizedCoefficients(optimized_coeff, polynomials_);
}


ComSpline4::VecScalar
ComSpline4::ExpressCogPosThroughABCD(double t_local, int id, Coords dim) const
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

ComSpline4::VecScalar
ComSpline4::ExpressCogVelThroughABCD (double t_local, int id,
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


ComSpline4::VecScalar
ComSpline4::ExpressCogAccThroughABCD(double t_local, int id, Coords dim) const
{
  VecScalar acc(GetTotalFreeCoeff());

  int idx = Index(id,dim,A);
  acc.v.middleCols<kFreeCoeffPerSpline>(idx) = ExpressCogAccThroughABCD(t_local);

  return acc;
}


ComSpline4::VecABCD
ComSpline4::ExpressCogAccThroughABCD(double t_local) const
{
  VecABCD acc;

  // x_acc = 20at^3 + 12bt^2 + 6ct   + 2d
  acc(A)   = 20.0 * std::pow(t_local,3);
  acc(B)   = 12.0 * std::pow(t_local,2);
  acc(C)   =  6.0 * t_local;
  acc(D)   =  2.0;

  return acc;
}

ComSpline4::VecScalar
ComSpline4::ExpressCogJerkThroughABCD (double t_local, int id,
                                                      Coords dim) const
{
  VecScalar jerk(GetTotalFreeCoeff());

  // x_jerk = 60at^2 +   24bt +  6c
  jerk.v(Index(id,dim,A))   = 60 * std::pow(t_local,2);
  jerk.v(Index(id,dim,B))   = 24 * std::pow(t_local,1);
  jerk.v(Index(id,dim,C))   = 6;

  return jerk;
}


int ComSpline4::GetTotalFreeCoeff() const
{
  CheckIfSplinesInitialized();
  return polynomials_.size() * kFreeCoeffPerSpline * kDim2d;
}

ComSpline4::VectorXd
ComSpline4::GetCoeffients () const
{
  VectorXd x_abcd(GetTotalFreeCoeff());

  for (const auto& s : polynomials_)
    for (auto dim : Coords2DArray)
      for (auto coeff : SplineCoeffArray)
        x_abcd[Index(s.GetId(), dim, coeff)] = s.GetCoefficient(dim, coeff);

  return x_abcd;
}

int ComSpline4::Index(int spline, Coords dim, SplineCoeff coeff) const
{
  return kFreeCoeffPerSpline * kDim2d * spline + kFreeCoeffPerSpline * dim + coeff;
}

ComSpline4::VecScalar
ComSpline4::GetECoefficient(int spline_id_k, Coords dim) const
{
  CheckIfSplinesInitialized();
  return relationship_e_to_abcd_.at(dim).GetRow(spline_id_k);
}


ComSpline4::VecScalar
ComSpline4::GetFCoefficient(int spline_id_k, Coords dim) const
{
  CheckIfSplinesInitialized();
  return relationship_f_to_abdc_.at(dim).GetRow(spline_id_k);
}



ComSpline4::MatVec
ComSpline4::DescribeEByABCD(Coords dim, double start_cog_v) const
{
  MatVec e_coeff(polynomials_.size(), GetTotalFreeCoeff());
  e_coeff.v[0] = start_cog_v;

  for (uint k=1; k<polynomials_.size(); ++k)
  {
    int kprev = k-1;

    // velocity at beginning of previous spline (e_prev)
    e_coeff.M.row(k) = e_coeff.M.row(kprev);
    e_coeff.v[k]     = e_coeff.v[kprev];

    // velocity change over previous spline due to a,b,c,d
    double Tkprev = polynomials_.at(kprev).GetDuration();
    e_coeff.M(k, Index(kprev,dim,A)) += 5*std::pow(Tkprev,4);
    e_coeff.M(k, Index(kprev,dim,B)) += 4*std::pow(Tkprev,3);
    e_coeff.M(k, Index(kprev,dim,C)) += 3*std::pow(Tkprev,2);
    e_coeff.M(k, Index(kprev,dim,D)) += 2*std::pow(Tkprev,1);
  }

  return e_coeff;
}


ComSpline4::MatVec
ComSpline4::DescribeFByABCD(Coords dim, double start_cog_p,
                                           double start_cog_v) const
{
  MatVec e_coeff = DescribeEByABCD(dim, start_cog_v);

  MatVec f_coeff(polynomials_.size(), GetTotalFreeCoeff());
  f_coeff.v[0] = start_cog_p;

  for (uint k=1; k<polynomials_.size(); ++k)
  {
    int kprev = k-1;
    // position at start of previous spline (=f_prev)
    f_coeff.M.row(k) = f_coeff.M.row(kprev);
    f_coeff.v[k]     = f_coeff.v[kprev];

    double Tkprev    = polynomials_.at(kprev).GetDuration();

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
ComSpline4::AddOptimizedCoefficients(
    const Eigen::VectorXd& optimized_coeff,
    VecPolynomials& splines) const
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
ComSpline4::SetEndAtStart ()
{
  const Vector2d& start_com_v = polynomials_.front().GetState(xpp::utils::kVel,0.0);
  double T = polynomials_.front().GetDuration();

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

  SetCoefficients(abcd);
}

} /* namespace zmp */
} /* namespace xpp */

