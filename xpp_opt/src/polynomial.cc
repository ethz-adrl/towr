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

#include <../include/xpp_opt/polynomial.h>

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace xpp {

using namespace opt;


Polynomial::Polynomial (int order, int dim)
{
  int n_coeff = order+1;
  for (int c=A; c<n_coeff; ++c) {
    coeff_ids_.push_back(static_cast<PolynomialCoeff>(c));
    coeff_.push_back(VectorXd::Zero(dim));
  }

  n_dim_ = dim;
  int n_variables = n_coeff*n_dim_;
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

CubicHermitePoly::~CubicHermitePoly ()
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
                                    MotionDerivative node_value,
                                    double t_local) const
{
  switch (dxdt) {
    case kPos:
      return GetDerivativeOfPosWrt(side, node_value, t_local);
    case kVel:
      return GetDerivativeOfVelWrt(side, node_value, t_local);
    case kAcc:
      return GetDerivativeOfAccWrt(side, node_value, t_local);
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
  double t3 = std::pow(t,3);
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
  double t2 = std::pow(t,2);
  double t3 = std::pow(t,3);
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

//  return   (t3*(v0 + v1))/T3
//         - (t2*(2*v0 + v1))/T2;
//         - (3*t3*(2*p0 - 2*p1 + T*v0 + T*v1))/T4
//         + (2*t2*(3*p0 - 3*p1 + 2*T*v0 + T*v1))/T3
}





PolynomialVars::PolynomialVars (const std::string& id, const Polynomial::Ptr& poly)
    : Variable(-1, id)
{
  polynomial_ = poly;
  SetRows(polynomial_->GetCoeffCount());
}

int
PolynomialVars::Index(PolynomialCoeff coeff, int dim) const
{
  return coeff*polynomial_->GetDimCount() + dim;
}

VectorXd
PolynomialVars::GetValues () const
{
  VectorXd x(GetRows());
  for (auto c : polynomial_->GetCoeffIds())
    for (int dim=0; dim<polynomial_->GetDimCount(); ++dim) {
      double val = polynomial_->GetCoefficients(c)(dim);
      x(Index(c, dim)) = val;
    }

  return x;
}

void
PolynomialVars::SetVariables (const VectorXd& x)
{
  for (auto c : polynomial_->GetCoeffIds()) {
    for (int dim=0; dim<polynomial_->GetDimCount(); ++dim) {
      double val = x(Index(c, dim));
      polynomial_->SetCoefficient(c, dim, val);
    }
  }
}

PolynomialVars::Jacobian
PolynomialVars::GetJacobian (double t_local, MotionDerivative dxdt) const
{
  int n_dim = polynomial_->GetDimCount();
  Jacobian jac(n_dim, GetRows());

  for (int dim=0; dim<n_dim; ++dim) {
    for (PolynomialCoeff c : polynomial_->GetCoeffIds()) {

      int idx = Index(c, dim);
      jac.insert(dim, idx) = polynomial_->GetDerivativeWrtCoeff(t_local, dxdt, c);
    }
  }

  return jac;
}

//VecBound
//PolynomialVars::GetBounds () const
//{
//  VecBound bounds(GetRows(), kNoBound_);
//
//  for (auto c : polynomial_->GetCoeffIds()) {
//    for (int dim=0; dim<polynomial_->GetDimCount(); ++dim) {
//      double idx = Index(c, dim);
//      if (c==B && dim==Z)
//        bounds.at(idx) = kEqualityBound_;
//    }
//  }
//  return bounds;
//}




//double
//CubicHermitePoly::GetDerivativeOfPosWrtStartPos (double t) const
//{
//  return (2*std::pow(t,3))/T3 - (3*std::pow(t,2))/T2 + 1;
//}
//
//double
//CubicHermitePoly::GetDerivativeOfPosWrtStartVel (double t) const
//{
//  return t - (2*std::pow(t,2))/T + std::pow(t,3)/T2;
//}
//
//double
//CubicHermitePoly::GetDerivativeOfPosWrtEndPos (double t) const
//{
//  return (3*std::pow(t,2))/T2 - (2*std::pow(t,3))/T3;
//}
//
//double
//CubicHermitePoly::GetDerivativeOfPosWrtEndVel (double t) const
//{
//  return std::pow(t,3)/T2 - std::pow(t,2)/T;
//}


//void
//ConstantPolynomial::SetPolynomialCoefficients (double T,
//                                               const StateLinXd& start,
//                                               const StateLinXd& end)
//{
//  coeff_[A] = start.p_;
//}
//
//double
//ConstantPolynomial::GetDerivativeOfPosWrtPos (double t, PointType p) const
//{
//  switch (p) {
//    case Start: return 1.0;
//    case Goal:  return 0.0;
//    default: assert(false); // point type not defined
//  }
//}
//
//void LinearPolynomial::SetPolynomialCoefficients(double T, const StateLinXd& start, const StateLinXd& end)
//{
//  coeff_[A] = start.p_;
//  coeff_[B] = (end.p_ - start.p_) / T;
//}
//
//double
//LinearPolynomial::GetDerivativeOfPosWrtPos (double t, PointType p) const
//{
//  switch (p) {
//    case Start: return 1.0-t/duration;
//    case Goal:  return    +t/duration;
//    default: assert(false); // point type not defined
//  }
//}

//void CubicPolynomial::SetPolynomialCoefficients(double T, const StateLinXd& start, const StateLinXd& end)
//{
//  double T1 = T;
//  double T2 = T1 * T1;
//  double T3 = T1 * T2;
//
//  coeff_[A] = start.p_;
//  coeff_[B] = start.v_;
//  coeff_[C] = -( 3*(start.p_ - end.p_) +  T1*(2*start.v_ + end.v_) ) / T2;
//  coeff_[D] =  ( 2*(start.p_ - end.p_) +  T1*(  start.v_ + end.v_) ) / T3;
//}
//
//double
//CubicPolynomial::GetDerivativeOfPosWrtPos (double t, PointType p) const
//{
//  double T2 = duration*duration;
//  double T3 = T2*duration;
//
//  switch (p) {
//    case Start: return (2*t*t*t)/T3 - (3*t*t)/T2 + 1;
//    case Goal:  return (3*t*t)/T2   - (2*t*t*t)/T3;
//    default: assert(false); // point type not defined
//  }
//}
//
//void QuarticPolynomial::SetPolynomialCoefficients(double T, const StateLinXd& start, const StateLinXd& end)
//{
//  // only uses position, velocity and initial acceleration
//  double T1 = T;
//  double T2 = T1 * T1;
//  double T3 = T1 * T2;
//  double T4 = T1 * T3;
//
//  coeff_[A] = start.p_;
//  coeff_[B] = start.v_;
//  coeff_[C] = start.a_/2;
//  coeff_[D] = -(4*start.p_ - 4*end.p_ + 3*T1*start.v_ +   T1*end.v_ + T2*start.a_)/T3;
//  coeff_[E] =  (6*start.p_ - 6*end.p_ + 4*T1*start.v_ + 2*T1*end.v_ + T2*start.a_)/(2*T4);
//}

//void QuinticPolynomial::SetPolynomialCoefficients(double T, const StateLinXd& start, const StateLinXd& end)
//{
//  double T1 = T;
//  double T2 = T1 * T1;
//  double T3 = T1 * T2;
//  double T4 = T1 * T3;
//  double T5 = T1 * T4;
//
//  coeff_[A] = start.p_;
//  coeff_[B] = start.v_;
//  coeff_[C] = start.a_ / 2.;
//  coeff_[D] =  (-20*start.p_ + 20*end.p_ + T1*(-3*start.a_*T1 +   end.a_*T1 - 12* start.v_ -  8*end.v_))  / (2*T3);
//  coeff_[E] =  ( 30*start.p_ - 30*end.p_ + T1*( 3*start.a_*T1 - 2*end.a_*T1 + 16* start.v_ + 14*end.v_))  / (2*T4);
//  coeff_[F] = -( 12*start.p_ - 12*end.p_ + T1*(   start.a_*T1 -   end.a_*T1 +  6*(start.v_ +    end.v_))) / (2*T5);
//}
//
//
//void
//LiftHeightPolynomial::SetPolynomialCoefficients (
//    double T, const StateLinXd& start, const StateLinXd& end)
//{
//  VectorXd h_ = height_*VectorXd::Ones(start.kNumDim);
//
//  double n2 = std::pow(n_,2);
//  double n3 = std::pow(n_,3);
//  double n4 = std::pow(n_,4);
//  double n5 = std::pow(n_,5);
//
//  double T1 = T;
//  double T2 = T1 * T1;
//  double T3 = T1 * T2;
//  double T4 = T1 * T3;
//  double T5 = T1 * T4;
//
//  // see matlab script "swingleg_z_height.m" for generation of these values
//  coeff_[A] =  start.p_;
//  coeff_[B].setZero();
//  coeff_[C] =  (6*end.p_ - 6*start.p_ - 15*n_*end.p_ + 15*n_*start.p_ + 2*h_*n4 - h_*n5 + 10*n2*end.p_ - 10*n2*start.p_)/(- n3*T2 + 4*n2*T2 - 5*n_*T2 + 2*T2);
//  coeff_[D] =  (2*(2*end.p_ - 2*start.p_ - 5*n_*end.p_ + 5*n_*start.p_ + 2*h_*n4 - h_*n5 + 5*n3*end.p_ - 5*n3*start.p_))/((n_ - 2)*(n2*T3 - 2*n_*T3 + T3));
//  coeff_[E] = -(2*h_*n4 - h_*n5 - 10*n2*end.p_ + 15*n3*end.p_ + 10*n2*start.p_ - 15*n3*start.p_)/(T4*(n_ - 2)*(n2 - 2*n_ + 1));
//  coeff_[F] = -(2*(2*n2*end.p_ - 3*n3*end.p_ - 2*n2*start.p_ + 3*n3*start.p_))/(T5*(n_ - 2)*(n2 - 2*n_ + 1));
//}

} // namespace xpp
