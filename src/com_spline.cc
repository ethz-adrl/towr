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

ComSpline::ComSpline () : Component(-1, "com_spline")
{
}

ComSpline::~ComSpline ()
{
}

void
ComSpline::Init (double t_global, double duration_polynomial)
{
  double t_left = t_global;
  while (t_left > duration_polynomial) {
    polynomials_.push_back(PolyXdT(duration_polynomial));
    t_left -= duration_polynomial;
  }

  // final polynomial has different duration
  polynomials_.push_back(PolyXdT(t_left));

  SetRows(polynomials_.size() * NumFreeCoeffPerSpline() * dim_.size());

  // zmp_ remove
//  z_height_ = height;
}

StateLin3d ComSpline::GetCom(double t_global) const
{
  // zmp_ remove
  StateLin3d com;

  StateLin2d xy = PolyHelpers::GetPoint(t_global, polynomials_);
  com.SetDimension(xy.Get1d(X),X);
  com.SetDimension(xy.Get1d(Y),Y);
  com.p.z() = z_height_;

  return com;
}

double ComSpline::GetTotalTime() const
{
  return PolyHelpers::GetTotalTime(polynomials_);
}

int
ComSpline::Index (int poly, Coords3D dim, PolyCoeff coeff) const
{
  return NumFreeCoeffPerSpline() * dim_.size() * poly
       + NumFreeCoeffPerSpline() * dim
       + coeff;
}

VectorXd
ComSpline::GetValues () const
{
  VectorXd x_abcd(GetRows());

  int i=0;
  for (const auto& s : polynomials_) {
    for (auto dim : dim_)
      for (auto coeff :  s.GetDim(dim).GetCoeffIds())
        x_abcd[Index(i, dim, coeff)] = s.GetCoefficient(dim, coeff);
    i++;
  }

  return x_abcd;
}

VecScalar
ComSpline::GetLinearApproxWrtCoeff (double t_global, MotionDerivative dxdt, Coords3D dim) const
{
  VecScalar linear_approx; // at current coefficient values

  linear_approx.v = GetJacobian(t_global, dxdt, dim);
  linear_approx.s = GetCom(t_global).GetByIndex(dxdt, dim);

  return linear_approx;
}

ComSpline::JacobianRow
ComSpline::GetJacobian (double t_global, MotionDerivative deriv, Coords3D dim) const
{
  int id         = PolyHelpers::GetPolynomialID(t_global, polynomials_);
  double t_local = PolyHelpers::GetLocalTime(t_global, polynomials_);

  return GetJacobianWrtCoeffAtPolynomial(deriv, t_local, id, dim);
}

ComSpline::JacobianRow
ComSpline::GetJacobianWrtCoeffAtPolynomial (MotionDerivative deriv, double t_local,
                                            int id, Coords3D dim) const
{
  JacobianRow jac(1, GetRows());
  auto polynomial = polynomials_.at(id).GetDim(dim);

  for (auto coeff : polynomial.GetCoeffIds()) {
    double val = polynomial.GetDerivativeWrtCoeff(deriv, coeff, t_local);
    int idx = Index(id,dim,coeff);
    jac.insert(idx) = val;
  }

  return jac;
}

void
ComSpline::SetValues (const VectorXd& optimized_coeff)
{
  for (size_t p=0; p<polynomials_.size(); ++p) {
    auto& poly = polynomials_.at(p);
    for (const Coords3D dim : dim_)
      for (auto c : poly.GetDim(dim).GetCoeffIds())
        poly.SetCoefficients(dim, c, optimized_coeff[Index(p,dim,c)]);
  }
}

int
ComSpline::NumFreeCoeffPerSpline () const
{
  // careful: assuming all polynomials and dimensions are same polynomial.
  return polynomials_.front().GetDim(X).GetCoeffIds().size();
}

} /* namespace opt */
} /* namespace xpp */
