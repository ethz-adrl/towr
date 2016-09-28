/**
@file    com_spline6.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   Defines ComSpline6, which realizes a ComSpline
 */

#include <xpp/zmp/com_spline6.h>

namespace xpp {
namespace zmp {

using namespace xpp::utils; // X,Y,Z

ComSpline6::ComSpline6 ()
{
  // TODO Auto-generated constructor stub
}

ComSpline6::~ComSpline6 ()
{
  // TODO Auto-generated destructor stub
}

ComSpline6::PtrClone
ComSpline6::clone () const
{
  return PtrClone(new ComSpline6(*this));
}

ComSpline6::Derivatives
ComSpline6::GetInitialFreeMotions () const
{
  return {kPos, kVel, kAcc};
}

ComSpline6::Derivatives
ComSpline6::GetJunctionFreeMotions () const
{
  return {kPos, kVel, kAcc, kJerk};
}

void
ComSpline6::SetCoefficients (const VectorXd& optimized_coeff)
{
  CheckIfSplinesInitialized();

  for (size_t p=0; p<polynomials_.size(); ++p) {
    CoeffValues coeff_values;

    for (const Coords3D dim : {X,Y}) {
      double* cv = (dim == X) ? coeff_values.x : coeff_values.y;
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

void
ComSpline6::GetJacobianPos (double t_poly, int id, Coords3D dim, JacobianRow& jac) const
{
  // x_pos = at^5 +   bt^4 +  ct^3 + dt*2 + et + f
  jac.insert(Index(id,dim,A))   = std::pow(t_poly,5);
  jac.insert(Index(id,dim,B))   = std::pow(t_poly,4);
  jac.insert(Index(id,dim,C))   = std::pow(t_poly,3);
  jac.insert(Index(id,dim,D))   = std::pow(t_poly,2);
  jac.insert(Index(id,dim,E))   = t_poly;
  jac.insert(Index(id,dim,F))   = 1;
}

void
ComSpline6::GetJacobianVel (double t_poly, int id, Coords3D dim, JacobianRow& jac) const
{
  // x_vel = 5at^4 +   4bt^3 +  3ct^2 + 2dt + e
  jac.insert(Index(id,dim,A))   = 5 * std::pow(t_poly,4);
  jac.insert(Index(id,dim,B))   = 4 * std::pow(t_poly,3);
  jac.insert(Index(id,dim,C))   = 3 * std::pow(t_poly,2);
  jac.insert(Index(id,dim,D))   = 2 * t_poly;
  jac.insert(Index(id,dim,E))   = 1;
}

void
ComSpline6::GetJacobianAcc (double t_poly, int id, Coords3D dim, JacobianRow& jac) const
{
  // x_acc = 20at^3 + 12bt^2 + 6ct   + 2d
  jac.insert(Index(id,dim,A))   = 20.0 * std::pow(t_poly,3);
  jac.insert(Index(id,dim,B))   = 12.0 * std::pow(t_poly,2);
  jac.insert(Index(id,dim,C))   =  6.0 * t_poly;
  jac.insert(Index(id,dim,D))   =  2.0;
}

void
ComSpline6::GetJacobianJerk (double t_poly, int id, Coords3D dim, JacobianRow& jac) const
{
  // x_jerk = 60at^2 +   24bt +  6c
  jac.insert(Index(id,dim,A))   = 60 * std::pow(t_poly,2);
  jac.insert(Index(id,dim,B))   = 24 * std::pow(t_poly,1);
  jac.insert(Index(id,dim,C))   = 6;
}

} // namespace zmp
} // namespace xpp


