/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/com_spline.h>

#include <array>
#include <cassert>
#include <cmath>
#include <stddef.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/opt/polynomial_xd.h>

namespace xpp {
namespace opt {

ComSpline::ComSpline ()
{
}

ComSpline::~ComSpline ()
{
}

void
ComSpline::Init (double t_global, int polynomials_per_second)
{
  int n_polyomials = std::ceil(t_global*polynomials_per_second);

  for (int i=0; i<n_polyomials; ++i)
    polynomials_.push_back(ComPolynomial(i, t_global/n_polyomials));

  SetVariableCount();
}

//void
//ComSpline::Init (const MotionPhases& phases, int polynomials_per_second)
//{
//  int id=0;
//  for (const auto& phase : phases) {
//    double T = phase.duration_;
//    int polys_per_phase = std::ceil(T*polynomials_per_second);
//    for (int i=0; i<polys_per_phase; ++i)
//      polynomials_.push_back(ComPolynomial(id++, T/polys_per_phase));
//  }
//
//  splines_initialized_ = true;
//}

StateLin2d ComSpline::GetCom(double t_global) const
{
  return ComPolynomialHelpers::GetCOM(t_global, polynomials_);
}

double ComSpline::GetTotalTime() const
{
  return ComPolynomialHelpers::GetTotalTime(polynomials_);
}

int
ComSpline::Index (int poly, Coords3D dim, PolyCoeff coeff) const
{
  return NumFreeCoeffPerSpline() * kDim2d * poly
       + NumFreeCoeffPerSpline() * dim
       + coeff;
}

int
ComSpline::GetTotalFreeCoeff () const
{
  return polynomials_.size() * NumFreeCoeffPerSpline() * kDim2d;
}

VectorXd
ComSpline::GetXYSplineCoeffients () const
{
  VectorXd x_abcd(GetTotalFreeCoeff());

  for (const auto& s : polynomials_)
    for (auto dim : { X, Y })
      for (auto coeff :  s.GetDim(dim).GetAllCoefficients())
        x_abcd[Index(s.GetId(), dim, coeff)] = s.GetCoefficient(dim, coeff);

  return x_abcd;
}

ComSpline::JacobianRow
ComSpline::GetJacobian (double t_global, MotionDerivative deriv, Coords3D dim) const
{
  int id         = ComPolynomialHelpers::GetPolynomialID(t_global, polynomials_);
  double t_local = ComPolynomialHelpers::GetLocalTime(t_global, polynomials_);

  return GetJacobianWrtCoeffAtPolynomial(deriv, t_local, id, dim);
}

ComSpline::JacobianRow
ComSpline::GetJacobianWrtCoeffAtPolynomial (MotionDerivative deriv, double t_local,
                                            int id, Coords3D dim) const
{
  assert(0 <= id && id <= polynomials_.back().GetId());

  JacobianRow jac(1, GetTotalFreeCoeff());
  auto polynomial = polynomials_.at(id).GetDim(dim);

  for (auto coeff : polynomial.GetAllCoefficients()) {
    double val = polynomial.GetDerivativeWrtCoeff(deriv, coeff, t_local);
    int idx = Index(id,dim,coeff);
    jac.insert(idx) = val;
  }

  return jac;
}

void
ComSpline::SetSplineXYCoefficients (const VectorXd& optimized_coeff)
{
  for (size_t p=0; p<polynomials_.size(); ++p)
    for (const Coords3D dim : {X,Y})
      for (auto c : Polynomial::kAllSplineCoeff)
        polynomials_.at(p).SetCoefficients(dim, c, optimized_coeff[Index(p,dim,c)]);
}

void
ComSpline::SetCoefficientsZero ()
{
  Eigen::VectorXd coeff(GetTotalFreeCoeff());
  SetSplineXYCoefficients(coeff.setZero());
}

int
ComSpline::NumFreeCoeffPerSpline () const
{
  return polynomials_.front().GetDim(X).GetAllCoefficients().size();
}

} /* namespace opt */
} /* namespace xpp */
