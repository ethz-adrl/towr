/**
@file   polynomial.cc
@author Alexander W. Winkler
@date   29.07.2016

@brief  A virtual class spliner with ready to use derived spliners

Spliners ready to use:
        - Linear Spliner
        - Cubic Spliner
        - Quintic Spliner
*/

#include <xpp/opt/polynomial.h>

#include <cassert>
#include <cmath>

namespace xpp {
namespace opt {

Polynomial::Polynomial (int order)
{
  int n_coeff = order+1;
  for (int c=A; c<n_coeff; ++c) {
    coeff_.push_back(0.0);
    coeff_ids_.push_back(static_cast<PolynomialCoeff>(c));
  }
}

void Polynomial::SetBoundary(double T, const StateLin1d& start_p, const StateLin1d& end_p)
{
  if(T <= 0.0)
    throw std::invalid_argument("Cannot create a Polynomial with zero or negative duration");

  duration = T;
  SetPolynomialCoefficients(T, start_p, end_p);
}

/**
 * The spliner always calculates the splines in the same way, but if the
 * spline coefficients are zero (as set by @ref Spliner()), the higher-order
 * terms have no effect.
 */
StateLin1d Polynomial::GetPoint(const double t) const
{
  // sanity checks
  if (t < 0.0)
    throw std::runtime_error("spliner.cc called with dt<0");

  StateLin1d out;

  for (auto d : {kPos, kVel, kAcc, kJerk}) {
    for (PolynomialCoeff c : GetCoeffIds()) {
      double val = GetDerivativeWrtCoeff(d, c, t)*coeff_[c];
      out.GetValue(d) += val;
    }
  }

  return out;
}

double
Polynomial::GetDerivativeWrtCoeff (MotionDerivative deriv, PolynomialCoeff c, double t) const
{
  if (c<deriv)  // risky/ugly business...
    return 0.0; // derivative not depended on this coefficient.

  switch (deriv) {
    case kPos:   return               std::pow(t,c);   break;
    case kVel:   return c*            std::pow(t,c-1); break;
    case kAcc:   return c*(c-1)*      std::pow(t,c-2); break;
    case kJerk:  return c*(c-1)*(c-2)*std::pow(t,c-3); break;
  }
}

double Polynomial::GetCoefficient(PolynomialCoeff coeff) const
{
  return coeff_[coeff];
}

void Polynomial::SetCoefficient(PolynomialCoeff coeff, double value)
{
  coeff_[coeff] = value;
}

Polynomial::CoeffVec
Polynomial::GetCoeffIds () const
{
  return coeff_ids_;
}

double Polynomial::GetDuration() const
{
  return duration;
}

void LinearPolynomial::SetPolynomialCoefficients(double T, const StateLin1d& start, const StateLin1d& end)
{
  coeff_[A] = start.p;
  coeff_[B] = (end.p - start.p) / T;
}

void CubicPolynomial::SetPolynomialCoefficients(double T, const StateLin1d& start, const StateLin1d& end)
{
  double T1 = T;
  double T2 = T1 * T1;
  double T3 = T1 * T2;

  coeff_[A] = start.p;
  coeff_[B] = start.v;
  coeff_[C] = -( 3*(start.p - end.p) +  T1*(2*start.v + end.v) ) / T2;
  coeff_[D] =  ( 2*(start.p - end.p) +  T1*(  start.v + end.v) ) / T3;
}

double
CubicPolynomial::GetDerivativeOfPosWrtPos (double t, PointType p) const
{
  double T2 = duration*duration;
  double T3 = T2*duration;

  switch (p) {
    case Start: return (2*t*t*t)/T3 - (3*t*t)/T2 + 1;
    case Goal:  return (3*t*t)/T2   - (2*t*t*t)/T3;
    default: assert(false); // point type not defined
  }
}

void QuinticPolynomial::SetPolynomialCoefficients(double T, const StateLin1d& start, const StateLin1d& end)
{
  double T1 = T;
  double T2 = T1 * T1;
  double T3 = T1 * T2;
  double T4 = T1 * T3;
  double T5 = T1 * T4;

  coeff_[A] = start.p;
  coeff_[B] = start.v;
  coeff_[C] = start.a / 2.;
  coeff_[D] =  (-20*start.p + 20*end.p + T1*(-3*start.a*T1 +   end.a*T1 - 12* start.v -  8*end.v))  / (2*T3);
  coeff_[E] =  ( 30*start.p - 30*end.p + T1*( 3*start.a*T1 - 2*end.a*T1 + 16* start.v + 14*end.v))  / (2*T4);
  coeff_[F] = -( 12*start.p - 12*end.p + T1*(   start.a*T1 -   end.a*T1 +  6*(start.v +    end.v))) / (2*T5);
}

void
LiftHeightPolynomial::SetShape (int n, double h)
{
  n_ = n;
  h_ = h;
}

void
LiftHeightPolynomial::SetPolynomialCoefficients (
    double T, const StateLin1d& start, const StateLin1d& end)
{
  double n2 = std::pow(n_,2);
  double n3 = std::pow(n_,3);
  double n4 = std::pow(n_,4);
  double n5 = std::pow(n_,5);

  double T1 = T;
  double T2 = T1 * T1;
  double T3 = T1 * T2;
  double T4 = T1 * T3;
  double T5 = T1 * T4;

  // see matlab script "swingleg_z_height.m" for generation of these values
  coeff_[A] =  start.p;
  coeff_[B] =  0.0;
  coeff_[C] =  (6*end.p - 6*start.p - 15*n_*end.p + 15*n_*start.p + 2*h_*n4 - h_*n5 + 10*n2*end.p - 10*n2*start.p)/(- n3*T2 + 4*n2*T2 - 5*n_*T2 + 2*T2);
  coeff_[D] =  (2*(2*end.p - 2*start.p - 5*n_*end.p + 5*n_*start.p + 2*h_*n4 - h_*n5 + 5*n3*end.p - 5*n3*start.p))/((n_ - 2)*(n2*T3 - 2*n_*T3 + T3));
  coeff_[E] = -(2*h_*n4 - h_*n5 - 10*n2*end.p + 15*n3*end.p + 10*n2*start.p - 15*n3*start.p)/(T4*(n_ - 2)*(n2 - 2*n_ + 1));
  coeff_[F] = -(2*(2*n2*end.p - 3*n3*end.p - 2*n2*start.p + 3*n3*start.p))/(T5*(n_ - 2)*(n2 - 2*n_ + 1));
}


} // namespace opt
} // namespace xpp
