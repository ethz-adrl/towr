
#include <towr/variables/polynomial.h>

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <Eigen/Eigen>


namespace towr {

using namespace xpp;


Polynomial::Polynomial (int order, int dim)
{
  int n_coeff = order+1;
  for (int c=A; c<n_coeff; ++c) {
    coeff_ids_.push_back(static_cast<PolynomialCoeff>(c));
    coeff_.push_back(VectorXd::Zero(dim));
  }

  n_dim_ = dim;
}


VectorXd
Polynomial::GetCoefficients (PolynomialCoeff c) const
{
  return coeff_.at(c);
}

void
Polynomial::SetCoefficient (PolynomialCoeff coeff, const VectorXd& value)
{
  coeff_.at(coeff) = value;
}

StateLinXd Polynomial::GetPoint(double t_local) const
{
  // sanity checks
  if (t_local < 0.0)
    throw std::runtime_error("spliner.cc called with dt<0");

  StateLinXd out(n_dim_);

  for (auto d : {kPos, kVel, kAcc})
    for (PolynomialCoeff c : coeff_ids_)
      out.GetByIndex(d) += GetDerivativeWrtCoeff(t_local, d, c)*coeff_.at(c);//GetCoefficients(c);

  return out;
}

void
Polynomial::SetCoefficient (PolynomialCoeff c, int dim, double value)
{
  coeff_.at(c)(dim) = value;
}

double
Polynomial::GetDerivativeWrtCoeff (double t, MotionDerivative deriv, PolynomialCoeff c) const
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


CubicHermitePoly::CubicHermitePoly (int dim) : Polynomial(3,dim)
{
}

void
CubicHermitePoly::SetNodes (const Node& n0, const Node& n1, double T)
{
  coeff_[A] =  n0.at(kPos);
  coeff_[B] =  n0.at(kVel);
  coeff_[C] = -( 3*(n0.at(kPos) - n1.at(kPos)) +  T*(2*n0.at(kVel) + n1.at(kVel)) ) / std::pow(T,2);
  coeff_[D] =  ( 2*(n0.at(kPos) - n1.at(kPos)) +  T*(  n0.at(kVel) + n1.at(kVel)) ) / std::pow(T,3);

  T_ = T;
  n0_ = n0;
  n1_ = n1;
}


double
CubicHermitePoly::GetDerivativeOf (MotionDerivative dxdt, Side side,
                                   MotionDerivative node_derivative, // pos or velocity node
                                   double t_local) const
{
  switch (dxdt) {
    case kPos:
      return GetDerivativeOfPosWrt(side, node_derivative, t_local);
    case kVel:
      return GetDerivativeOfVelWrt(side, node_derivative, t_local);
    case kAcc:
      return GetDerivativeOfAccWrt(side, node_derivative, t_local);
    default:
      assert(false); // derivative not yet implemented
  }
}


double
CubicHermitePoly::GetDerivativeOfPosWrt (Side side, MotionDerivative node_value,
                                         double t) const
{
  double t2 = std::pow(t,2);
  double t3 = std::pow(t,3);
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);

  switch (side) {
    case Start:
      switch (node_value) {
        case kPos: return (2*t3)/T3 - (3*t2)/T2 + 1;
        case kVel: return t - (2*t2)/T + t3/T2;
      }

    case End:
      switch (node_value) {
        case kPos: return (3*t2)/T2 - (2*t3)/T3;
        case kVel: return t3/T2 - t2/T;
      }

    default: assert(false);
  }
}

double
CubicHermitePoly::GetDerivativeOfVelWrt (Side side, MotionDerivative node_value,
                                         double t) const
{
  double t2 = std::pow(t,2);
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);

  switch (side) {
    case Start:
      switch (node_value) {
        case kPos: return (6*t2)/T3 - (6*t)/T2;
        case kVel: return (3*t2)/T2 - (4*t)/T + 1;
      }

    case End:
      switch (node_value) {
        case kPos: return (6*t)/T2 - (6*t2)/T3;
        case kVel: return (3*t2)/T2 - (2*t)/T;
      }

    default: assert(false);
  }
}


double
CubicHermitePoly::GetDerivativeOfAccWrt (Side side, MotionDerivative node_value,
                                         double t) const
{
  double T  = T_;
  double T2 = std::pow(T_,2);
  double T3 = std::pow(T_,3);

  switch (side) {
    case Start:
      switch (node_value) {
        case kPos: return (12*t)/T3 - 6/T2;
        case kVel: return (6*t)/T2 - 4/T;
      }

    case End:
      switch (node_value) {
        case kPos: return 6/T2 - (12*t)/T3;
        case kVel: return (6*t)/T2 - 2/T;
      }

    default: assert(false);
  }
}

VectorXd
CubicHermitePoly::GetDerivativeOfPosWrtDuration(double t) const
{
  VectorXd x0 = n0_.at(kPos);
  VectorXd x1 = n1_.at(kPos);
  VectorXd v0 = n0_.at(kVel);
  VectorXd v1 = n1_.at(kVel);

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
