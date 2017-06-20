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
ComSpline::Init (double t_global, double dt, const State& com_start)
{
  double t_left = t_global;
  while (t_left > 0.0) {
    double duration = t_left>dt?  dt : t_left;
    auto p = std::make_shared<QuarticPolynomial>();
    p->SetBoundary(duration, com_start, com_start);
    polynomials_.push_back(p);
    t_left -= dt;
  }

  spline_.SetSegmentsPtr(polynomials_);
  SetRows(GetTotalFreeCoeff());
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
  return GetFreeCoeffPerPoly() * dim_.size() * poly
       + GetFreeCoeffPerPoly() * dim
       + coeff;
}

VectorXd
ComSpline::GetValues () const
{
  VectorXd x_abcd(GetRows());

  int i=0;
  for (const auto& s : polynomials_) {
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
  int id         = spline_.GetSegmentID(t_global);
  double t_local = spline_.GetLocalTime(t_global);

  return GetJacobianWrtCoeffAtPolynomial(deriv, t_local, id, dim);
}

JacobianRow
ComSpline::GetJacobianWrtCoeffAtPolynomial (MotionDerivative deriv, double t_local,
                                            int id, Coords3D dim) const
{
  JacobianRow jac(1, GetRows());
  auto polynomial = polynomials_.at(id);

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
  for (size_t p=0; p<polynomials_.size(); ++p) {
    auto& poly = polynomials_.at(p);
    for (const Coords3D dim : dim_)
      for (auto c : poly->GetCoeffIds())
        poly->SetCoefficient(dim, c, optimized_coeff[Index(p,dim,c)]);
  }
}

int
ComSpline::GetFreeCoeffPerPoly () const
{
  return polynomials_.front()->GetCoeffIds().size();
}

int
ComSpline::GetTotalFreeCoeff () const
{
  int n_polys = polynomials_.size();
  int n_dim   = dim_.size();

  return n_polys*GetFreeCoeffPerPoly()*n_dim;
}

} /* namespace opt */
} /* namespace xpp */
