/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/com_spline.h>

#include <stddef.h>
#include <string>
#include <Eigen/Sparse>

namespace xpp {
namespace opt {

ComSpline::ComSpline () : Component(-1, "com_spline")
{
  dim_ = State().GetDim(); // usually x,y,z
}

ComSpline::~ComSpline ()
{
}

void
ComSpline::Init (double t_global, double dt)
{
  double t_left = t_global;
  while (t_left > 0.0) {
    double duration = t_left>dt?  dt : t_left;
    auto p = std::make_shared<QuarticPolynomial>();
    p->SetBoundary(duration, State(), State());
    spline_.GetImpl().push_back(p);
    t_left -= dt;
  }

  SetRows(spline_.GetTotalFreeCoeff());
}

ComSpline::State
ComSpline::GetCom(double t_global) const
{
  return spline_.GetPoint(t_global);
}

double ComSpline::GetTotalTime() const
{
  return spline_.GetTotalTime();
}

int
ComSpline::Index (int poly, Coords3D dim, PolyCoeff coeff) const
{
  return spline_.GetFreeCoeffPerPoly() * dim_.size() * poly
       + spline_.GetFreeCoeffPerPoly() * dim
       + coeff;
}

VectorXd
ComSpline::GetValues () const
{
  VectorXd x_abcd(GetRows());

  int i=0;
  for (const auto& s : spline_.GetImpl()) {
    for (auto dim : dim_)
      for (auto coeff :  s->GetCoeffIds())
        x_abcd[Index(i, dim, coeff)] = s->GetCoefficient(dim, coeff);
    i++;
  }

  return x_abcd;
}

JacobianRow
ComSpline::GetJacobian (double t_global, MotionDerivative deriv, Coords3D dim) const
{
  int id         = spline_.GetPolynomialID(t_global);
  double t_local = spline_.GetLocalTime(t_global);

  return GetJacobianWrtCoeffAtPolynomial(deriv, t_local, id, dim);
}

JacobianRow
ComSpline::GetJacobianWrtCoeffAtPolynomial (MotionDerivative deriv, double t_local,
                                            int id, Coords3D dim) const
{
  JacobianRow jac(1, GetRows());
  auto polynomial = spline_.GetImpl().at(id);

  for (auto coeff : polynomial->GetCoeffIds()) {
    double val = polynomial->GetDerivativeWrtCoeff(deriv, coeff, t_local);
    int idx = Index(id,dim,coeff);
    jac.insert(idx) = val;
  }

  return jac;
}

void
ComSpline::SetValues (const VectorXd& optimized_coeff)
{
  for (size_t p=0; p<spline_.GetImpl().size(); ++p) {
    auto& poly = spline_.GetImpl().at(p);
    for (const Coords3D dim : dim_)
      for (auto c : poly->GetCoeffIds())
        poly->SetCoefficient(dim, c, optimized_coeff[Index(p,dim,c)]);
  }
}

} /* namespace opt */
} /* namespace xpp */
