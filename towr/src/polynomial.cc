/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/variables/polynomial.h>

#include <cassert>
#include <cmath>


namespace towr {

Polynomial::Polynomial (int order, int dim)
{
  int n_coeff = order+1;
  for (int c=A; c<n_coeff; ++c) {
    coeff_ids_.push_back(static_cast<Coefficients>(c));
    coeff_.push_back(VectorXd::Zero(dim));
  }
}

State Polynomial::GetPoint(double t_local) const
{
  // sanity checks
  if (t_local < 0.0)
    assert(false);//("spliner.cc called with dt<0")

  int n_dim = coeff_.front().size();
  State out(n_dim, 3);

  for (auto d : {kPos, kVel, kAcc})
    for (Coefficients c : coeff_ids_)
      out.at(d) += GetDerivativeWrtCoeff(t_local, d, c)*coeff_.at(c);

  return out;
}

double
Polynomial::GetDerivativeWrtCoeff (double t, Dx deriv, Coefficients c) const
{
  switch (deriv) {
    case kPos:   return               std::pow(t,c);         break;
    case kVel:   return c>=B? c*      std::pow(t,c-1) : 0.0; break;
    case kAcc:   return c>=C? c*(c-1)*std::pow(t,c-2) : 0.0; break;
    default: assert(false); // derivative not defined
  }
}



CubicHermitePolynomial::CubicHermitePolynomial (int dim)
    : Polynomial(3,dim),
      n0_(dim),
      n1_(dim)
{
  T_ = 0.0;
}

void
CubicHermitePolynomial::SetNodes (const Node& n0, const Node& n1)
{
  n0_ = n0;
  n1_ = n1;
}

void
CubicHermitePolynomial::SetDuration(double duration)
{
  T_ = duration;
}

void
CubicHermitePolynomial::UpdateCoeff()
{
  coeff_[A] =  n0_.p();
  coeff_[B] =  n0_.v();
  coeff_[C] = -( 3*(n0_.p() - n1_.p()) +  T_*(2*n0_.v() + n1_.v()) ) / std::pow(T_,2);
  coeff_[D] =  ( 2*(n0_.p() - n1_.p()) +  T_*(  n0_.v() + n1_.v()) ) / std::pow(T_,3);
}

double
CubicHermitePolynomial::GetDerivativeWrtStartNode (Dx dfdt,
                                                   Dx node_derivative, // pos or velocity node
                                                   double t_local) const
{
  switch (dfdt) {
    case kPos:
      return GetDerivativeOfPosWrtStartNode(node_derivative, t_local);
    case kVel:
      return GetDerivativeOfVelWrtStartNode(node_derivative, t_local);
    case kAcc:
      return GetDerivativeOfAccWrtStartNode(node_derivative, t_local);
    default:
      assert(false); // derivative not yet implemented
  }
}

double
CubicHermitePolynomial::GetDerivativeWrtEndNode (Dx dfdt,
                                                 Dx node_derivative, // pos or velocity node
                                                 double t_local) const
{
  switch (dfdt) {
    case kPos:
      return GetDerivativeOfPosWrtEndNode(node_derivative, t_local);
    case kVel:
      return GetDerivativeOfVelWrtEndNode(node_derivative, t_local);
    case kAcc:
      return GetDerivativeOfAccWrtEndNode(node_derivative, t_local);
    default:
      assert(false); // derivative not yet implemented
  }
}

double
CubicHermitePolynomial::GetDerivativeOfPosWrtStartNode(Dx node_value,
                                                       double t) const
{
  double t2 = std::pow(t,2);
  double t3 = std::pow(t,3);
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);

  switch (node_value) {
    case kPos: return (2*t3)/T3 - (3*t2)/T2 + 1;
    case kVel: return t - (2*t2)/T + t3/T2;
    default: assert(false); // only derivative wrt nodes values calculated
  }
}

double
CubicHermitePolynomial::GetDerivativeOfVelWrtStartNode (Dx node_value,
                                                        double t) const
{
  double t2 = std::pow(t,2);
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);

  switch (node_value) {
    case kPos: return (6*t2)/T3 - (6*t)/T2;
    case kVel: return (3*t2)/T2 - (4*t)/T + 1;
    default: assert(false); // only derivative wrt nodes values calculated
  }
}

double
CubicHermitePolynomial::GetDerivativeOfAccWrtStartNode (Dx node_value,
                                                        double t) const
{
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);

  switch (node_value) {
    case kPos: return (12*t)/T3 - 6/T2;
    case kVel: return (6*t)/T2 - 4/T;
    default: assert(false); // only derivative wrt nodes values calculated
  }
}

double
CubicHermitePolynomial::GetDerivativeOfPosWrtEndNode (Dx node_value,
                                                      double t) const
{
  double t2 = std::pow(t,2);
  double t3 = std::pow(t,3);
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);

  switch (node_value) {
    case kPos: return (3*t2)/T2 - (2*t3)/T3;
    case kVel: return t3/T2 - t2/T;
    default: assert(false); // only derivative wrt nodes values calculated
  }
}

double
CubicHermitePolynomial::GetDerivativeOfVelWrtEndNode (Dx node_value,
                                                      double t) const
{
  double t2 = std::pow(t,2);
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);

  switch (node_value) {
    case kPos: return (6*t)/T2 - (6*t2)/T3;
    case kVel: return (3*t2)/T2 - (2*t)/T;
    default: assert(false); // only derivative wrt nodes values calculated
  }
}

double
CubicHermitePolynomial::GetDerivativeOfAccWrtEndNode (Dx node_value,
                                                      double t) const
{
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);

  switch (node_value) {
    case kPos: return 6/T2 - (12*t)/T3;
    case kVel: return (6*t)/T2 - 2/T;
    default: assert(false); // only derivative wrt nodes values calculated
  }
}

Eigen::VectorXd
CubicHermitePolynomial::GetDerivativeOfPosWrtDuration(double t) const
{
  VectorXd x0 = n0_.p();
  VectorXd x1 = n1_.p();
  VectorXd v0 = n0_.v();
  VectorXd v1 = n1_.v();

  double t2 = std::pow(t,2);
  double t3 = std::pow(t,3);
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);
  double T4 = std::pow(T_,4);

  VectorXd deriv = (t3*(v0 + v1))/T3
                 - (t2*(2*v0 + v1))/T2
                 - (3*t3*(2*x0 - 2*x1 + T*v0 + T*v1))/T4
                 + (2*t2*(3*x0 - 3*x1 + 2*T*v0 + T*v1))/T3;

  return deriv;
}

} // namespace towr
