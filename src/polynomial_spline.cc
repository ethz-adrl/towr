/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/polynomial_spline.h>

#include <stddef.h>
#include <string>
#include <Eigen/Sparse>

namespace xpp {
namespace opt {

PolynomialSpline::PolynomialSpline (const std::string& component_name)
    : Component(-1, component_name)
{
}

PolynomialSpline::~PolynomialSpline ()
{
}

void
PolynomialSpline::Init (double t_global, double dt, const VectorXd& initial_pos)
{
  // initialize at com position with zero velocity & acceleration
  n_dim_ = initial_pos.rows();
  State initial_state(n_dim_);
  initial_state.p_ = initial_pos;

  double t_left = t_global;
  while (t_left > 0.0) {
    double duration = t_left>dt?  dt : t_left;
    auto p = std::make_shared<QuarticPolynomial>();
    p->SetBoundary(duration, initial_state, initial_state);
    polynomials_.push_back(p);
    t_left -= dt;
  }

  SetSegmentsPtr(polynomials_);
  SetRows(GetTotalFreeCoeff());
}

int
PolynomialSpline::Index (int poly, int dim, PolyCoeff coeff) const
{
  return GetFreeCoeffPerPoly() * n_dim_ * poly
       + GetFreeCoeffPerPoly() * dim
       + coeff;
}

VectorXd
PolynomialSpline::GetValues () const
{
  VectorXd x_abcd(GetRows());

  int i=0;
  for (const auto& s : polynomials_) {
    for (int dim = 0; dim<n_dim_; dim++)
      for (auto coeff :  s->GetCoeffIds())
        x_abcd[Index(i, dim, coeff)] = s->GetCoefficient(dim, coeff);
    i++;
  }

  return x_abcd;
}

JacobianRow
PolynomialSpline::GetJacobian (double t_global, MotionDerivative deriv, Coords3D dim) const
{
  int id         = GetSegmentID(t_global);
  double t_local = GetLocalTime(t_global);

  return GetJacobianWrtCoeffAtPolynomial(deriv, t_local, id, dim);
}

JacobianRow
PolynomialSpline::GetJacobianWrtCoeffAtPolynomial (MotionDerivative deriv, double t_local,
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
PolynomialSpline::SetValues (const VectorXd& optimized_coeff)
{
  for (size_t p=0; p<polynomials_.size(); ++p) {
    auto& poly = polynomials_.at(p);
    for (int dim = 0; dim < n_dim_; dim++)
      for (auto c : poly->GetCoeffIds())
        poly->SetCoefficient(dim, c, optimized_coeff[Index(p,dim,c)]);
  }
}

int
PolynomialSpline::GetFreeCoeffPerPoly () const
{
  return polynomials_.front()->GetCoeffIds().size();
}

int
PolynomialSpline::GetTotalFreeCoeff () const
{
  int n_polys = polynomials_.size();

  return n_polys*GetFreeCoeffPerPoly()*n_dim_;
}

} /* namespace opt */
} /* namespace xpp */
