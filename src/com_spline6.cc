/**
@file    com_spline6.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   Defines ComSpline6, which realizes a ComSpline
 */

#include <xpp/zmp/com_spline6.h>

namespace xpp {
namespace zmp {

using namespace xpp::utils::coords_wrapper; // X,Y,Z

ComSpline6::ComSpline6 ()
{
  // TODO Auto-generated constructor stub
}

ComSpline6::~ComSpline6 ()
{
  // TODO Auto-generated destructor stub
}

ComSpline6::UniquePtr
ComSpline6::clone () const
{
  return std::unique_ptr<ComSpline>(new ComSpline6(*this));
}

void
ComSpline6::Init (int step_count, const SplineTimes& times,
                  bool insert_initial_stance)
{
  ComSpline::Init(step_count, times, insert_initial_stance);

  // initialize all coefficients to zero
  Eigen::VectorXd abcd(GetTotalFreeCoeff());
  abcd.setZero();
  SetCoefficients(abcd);
}

ComSpline6::Derivatives
ComSpline6::GetInitialFreeMotions () const
{
  return {kPos, kVel, kAcc};
}

ComSpline6::Derivatives
ComSpline6::GetJunctionFreeMotions () const
{
  return {kPos, kVel, kAcc/*, kJerk*/};
}

ComSpline6::Derivatives
ComSpline6::GetFinalFreeMotions () const
{
  return {kPos, kVel, kAcc};
}

void
ComSpline6::SetCoefficients (const VectorXd& optimized_coeff)
{
  CheckIfSplinesInitialized();

  for (size_t p=0; p<polynomials_.size(); ++p) {
    CoeffValues coeff_values;

    for (const Coords3D dim : {X,Y}) {
      double* cv = (dim == xpp::utils::X) ? coeff_values.x : coeff_values.y;
      cv[A] = optimized_coeff[Index(p,dim,A)];
      cv[B] = optimized_coeff[Index(p,dim,B)];
      cv[C] = optimized_coeff[Index(p,dim,C)];
      cv[D] = optimized_coeff[Index(p,dim,D)];
      cv[E] = optimized_coeff[Index(p,dim,E)];
      cv[F] = optimized_coeff[Index(p,dim,F)];
    }

    polynomials_.at(p).SetSplineCoefficients(coeff_values);
  }
}

ComSpline6::VecScalar
ComSpline6::ExpressCogPosThroughABCD (double t_local, int id, Coords dim) const
{
  VecScalar pos(GetTotalFreeCoeff());

  // x_pos = at^5 +   bt^4 +  ct^3 + dt*2 + et + f
  pos.v(Index(id,dim,A))   = std::pow(t_local,5);
  pos.v(Index(id,dim,B))   = std::pow(t_local,4);
  pos.v(Index(id,dim,C))   = std::pow(t_local,3);
  pos.v(Index(id,dim,D))   = std::pow(t_local,2);
  pos.v(Index(id,dim,E))   = t_local;
  pos.v(Index(id,dim,F))   = 1;

  return pos;
}

ComSpline6::VecScalar
ComSpline6::ExpressCogVelThroughABCD (double t_local, int id, Coords dim) const
{
  VecScalar vel(GetTotalFreeCoeff());

  // x_vel = 5at^4 +   4bt^3 +  3ct^2 + 2dt + e
  vel.v(Index(id,dim,A))   = 5 * std::pow(t_local,4);
  vel.v(Index(id,dim,B))   = 4 * std::pow(t_local,3);
  vel.v(Index(id,dim,C))   = 3 * std::pow(t_local,2);
  vel.v(Index(id,dim,D))   = 2 * t_local;
  vel.v(Index(id,dim,E))   = 1;

  return vel;
}

ComSpline6::VecScalar
ComSpline6::ExpressCogAccThroughABCD (double t_local, int id, Coords dim) const
{
  VecScalar acc(GetTotalFreeCoeff());

  // x_acc = 20at^3 + 12bt^2 + 6ct   + 2d
  acc.v(Index(id,dim,A))   = 20.0 * std::pow(t_local,3);
  acc.v(Index(id,dim,B))   = 12.0 * std::pow(t_local,2);
  acc.v(Index(id,dim,C))   =  6.0 * t_local;
  acc.v(Index(id,dim,D))   =  2.0;

  return acc;
}

ComSpline6::VecScalar
ComSpline6::ExpressCogJerkThroughABCD (double t_local, int id, Coords dim) const
{
  VecScalar jerk(GetTotalFreeCoeff());

  // x_jerk = 60at^2 +   24bt +  6c
  jerk.v(Index(id,dim,A))   = 60 * std::pow(t_local,2);
  jerk.v(Index(id,dim,B))   = 24 * std::pow(t_local,1);
  jerk.v(Index(id,dim,C))   = 6;

  return jerk;
}

} // namespace zmp
} // namespace xpp


