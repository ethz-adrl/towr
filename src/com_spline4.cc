/**
@file    com_spline4.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   Defines ComSpline4, which realizes a ComSpline
 */

#include <xpp/opt/com_spline4.h>

#include <cassert>
#include <cmath>
#include <stddef.h>
#include <sys/types.h>
#include <Eigen/Dense>

#include <xpp/opt/com_polynomial_helpers.h>
#include <xpp/opt/polynomial_xd.h>

namespace xpp {
namespace opt {

ComSpline4::ComSpline4 ()
{
  // TODO Auto-generated constructor stub
}

ComSpline4::~ComSpline4 ()
{
  // TODO Auto-generated destructor stub
}

void
ComSpline4::SetStartPosVel (const Vector2d& start_cog_p, const Vector2d& start_cog_v)
{
 start_cog_p_ = start_cog_p;
 start_cog_v_ = start_cog_v;

 for (auto dim : {X,Y}) {
   jac_e_wrt_abcd_.at(dim) = CalcJacobianEWrtABCD(dim);
   jac_f_wrt_abcd_.at(dim) = CalcJacobianFWrtABCD(dim);
 }

 // initialize all other coefficients to zero
 Eigen::VectorXd abcd(GetTotalFreeCoeff());
 abcd.setZero();
 SetSplineXYCoefficients(abcd);
}

ComSpline4::Derivatives
ComSpline4::GetInitialFreeMotions () const
{
  return {}; //kAcc
}

ComSpline4::Derivatives
ComSpline4::GetJunctionFreeMotions () const
{
  return {kAcc};
//  return {kAcc /*, kJerk*/}; // jerk doesn't seem to work properly in spline junction
}

void
ComSpline4::SetSplineXYCoefficients(const VectorXd& optimized_coeff)
{
  CheckIfSplinesInitialized();
  assert(polynomials_.size() == GetTotalFreeCoeff()/kDim2d/4.0);

  Vector2d prev_pos = start_cog_p_;

  for (size_t k=0; k<polynomials_.size(); ++k) {
    for (auto dim : {X,Y}) {
      double cv[ComPolynomial::GetNumCoeff()];

      // fill in only first 4 optimized coefficients
      cv[Polynomial::PolynomialCoeff::A] = optimized_coeff[Index(k,dim,Polynomial::PolynomialCoeff::A)];
      cv[Polynomial::PolynomialCoeff::B] = optimized_coeff[Index(k,dim,Polynomial::PolynomialCoeff::B)];
      cv[Polynomial::PolynomialCoeff::C] = optimized_coeff[Index(k,dim,Polynomial::PolynomialCoeff::C)];
      cv[Polynomial::PolynomialCoeff::D] = optimized_coeff[Index(k,dim,Polynomial::PolynomialCoeff::D)];

      // calculate e and f coefficients from previous values
      JacobianRow jac_e = GetJacobianE(k, dim);
      JacobianRow jac_f = GetJacobianF(k, dim);

      if (k==0) { // first spline
        cv[Polynomial::PolynomialCoeff::E] = start_cog_v_[dim];
        cv[Polynomial::PolynomialCoeff::F] = start_cog_p_[dim];
      } else {
        // the value of the coefficients e and f for each spline comes from
        // a linear approximation around (a,b,c,d,a,...)=0 plus the initial
        // value of e and f for each spline k.
        double Tprev  = polynomials_.at(k-1).GetDuration();
        prev_pos[dim] += start_cog_v_[dim]*Tprev;

        cv[Polynomial::PolynomialCoeff::E] = (jac_e*optimized_coeff)[0] + start_cog_v_[dim];
        cv[Polynomial::PolynomialCoeff::F] = (jac_f*optimized_coeff)[0] + prev_pos[dim];
      }

      for (auto c : Polynomial::AllSplineCoeff) {
        polynomials_.at(k).SetCoefficients(dim, c, cv[c]);
      }

    } // dim:X..Y
  } // k=0..n_spline_infos_
}

void
ComSpline4::GetJacobianPos(double t_poly, int id, Coords3D dim, JacobianRow& jac) const
{
  JacobianRow jac_e = GetJacobianE(id, dim);
  JacobianRow jac_f = GetJacobianF(id, dim);

  // x_pos = at^5 +   bt^4 +  ct^3 + dt*2 + et + f
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::A)) = std::pow(t_poly,5);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::B)) = std::pow(t_poly,4);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::C)) = std::pow(t_poly,3);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::D)) = std::pow(t_poly,2);
  jac                          += t_poly*jac_e;
  jac                          += 1*jac_f;
}

void
ComSpline4::GetJacobianVel (double t_poly, int id, Coords3D dim, JacobianRow& jac) const
{
  JacobianRow jac_e = GetJacobianE(id, dim);

  // x_vel = 5at^4 +   4bt^3 +  3ct^2 + 2dt + e
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::A)) = 5 * std::pow(t_poly,4);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::B)) = 4 * std::pow(t_poly,3);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::C)) = 3 * std::pow(t_poly,2);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::D)) = 2 * std::pow(t_poly,1);
  jac                          += jac_e;
}

void
ComSpline4::GetJacobianAcc(double t_poly, int id, Coords3D dim, JacobianRow& jac) const
{
  // x_acc = 20at^3 + 12bt^2 + 6ct   + 2d
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::A)) = 20.0 * std::pow(t_poly,3);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::B)) = 12.0 * std::pow(t_poly,2);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::C)) =  6.0 * t_poly;
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::D)) =  2.0;
}

void
ComSpline4::GetJacobianJerk (double t_poly, int id, Coords3D dim, JacobianRow& jac) const
{
  // x_jerk = 60at^2 +   24bt +  6c
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::A)) = 60 * std::pow(t_poly,2);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::B)) = 24 * std::pow(t_poly,1);
  jac.insert(Index(id,dim,Polynomial::PolynomialCoeff::C)) = 6;
}

ComSpline4::JacobianRow
ComSpline4::GetJacobianE(int spline_id_k, Coords3D dim) const
{
  CheckIfSplinesInitialized();
  return jac_e_wrt_abcd_.at(dim).row(spline_id_k);
}

ComSpline4::JacobianRow
ComSpline4::GetJacobianF(int spline_id_k, Coords3D dim) const
{
  CheckIfSplinesInitialized();
  return jac_f_wrt_abcd_.at(dim).row(spline_id_k);
}

ComSpline4::JacobianEFWrtABCD
ComSpline4::CalcJacobianEWrtABCD (Coords3D dim) const
{
  JacobianEFWrtABCD jac(polynomials_.size(), GetTotalFreeCoeff());

  for (uint k=1; k<polynomials_.size(); ++k)
  {
    int kprev = k-1;

    // velocity at beginning of previous spline (e_prev)
    jac.row(k) = jac.row(kprev);

    // velocity change over previous spline due to a,b,c,d
    double Tkprev = polynomials_.at(kprev).GetDuration();
    jac.coeffRef(k, Index(kprev,dim,Polynomial::PolynomialCoeff::A)) += 5*std::pow(Tkprev,4);
    jac.coeffRef(k, Index(kprev,dim,Polynomial::PolynomialCoeff::B)) += 4*std::pow(Tkprev,3);
    jac.coeffRef(k, Index(kprev,dim,Polynomial::PolynomialCoeff::C)) += 3*std::pow(Tkprev,2);
    jac.coeffRef(k, Index(kprev,dim,Polynomial::PolynomialCoeff::D)) += 2*Tkprev;
  }

  return jac;
}

ComSpline4::JacobianEFWrtABCD
ComSpline4::CalcJacobianFWrtABCD (Coords3D dim) const
{
  JacobianEFWrtABCD jac(polynomials_.size(), GetTotalFreeCoeff());
  JacobianEFWrtABCD jac_e = CalcJacobianEWrtABCD(dim);

  for (uint k=1; k<polynomials_.size(); ++k)
  {
    int kprev = k-1;
    // position at start of previous spline (=f_prev)
    jac.row(k) = jac.row(kprev);

    double Tkprev = polynomials_.at(kprev).GetDuration();

    // position change over previous spline due to a,b,c,d
    jac.coeffRef(k, Index(kprev,dim,Polynomial::PolynomialCoeff::A)) += std::pow(Tkprev,5);
    jac.coeffRef(k, Index(kprev,dim,Polynomial::PolynomialCoeff::B)) += std::pow(Tkprev,4);
    jac.coeffRef(k, Index(kprev,dim,Polynomial::PolynomialCoeff::C)) += std::pow(Tkprev,3);
    jac.coeffRef(k, Index(kprev,dim,Polynomial::PolynomialCoeff::D)) += std::pow(Tkprev,2);
    // position change over previous spline due to e
    jac.row(k) += jac_e.row(kprev)*Tkprev;
  }

  return jac;
}

void
ComSpline4::SetEndAtStart ()
{
  const Vector2d& start_com_v = polynomials_.front().GetState(kVel,0.0);
  double T = polynomials_.front().GetDuration();

  Eigen::Matrix2d A; A << 3*T*T, 2*T, // velocity equal to zero
                          T*T*T, T*T; // position equal to initial
  Vector2d b(1,T);

  Vector2d c_and_d_x = A.lu().solve(-start_com_v.x()*b);
  Vector2d c_and_d_y = A.lu().solve(-start_com_v.y()*b);

  Eigen::VectorXd abcd(GetTotalFreeCoeff());
  abcd.setZero();
  abcd[Index(0,X,Polynomial::PolynomialCoeff::C)] = c_and_d_x(0);
  abcd[Index(0,X,Polynomial::PolynomialCoeff::D)] = c_and_d_x(1);
  abcd[Index(0,Y,Polynomial::PolynomialCoeff::C)] = c_and_d_y(0);
  abcd[Index(0,Y,Polynomial::PolynomialCoeff::D)] = c_and_d_y(1);

  SetSplineXYCoefficients(abcd);
}

} /* namespace zmp */
} /* namespace xpp */

