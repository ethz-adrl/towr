/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/variables/polynomial_spline.h>

#include <cassert>
#include <Eigen/Dense>

namespace xpp {
namespace opt {

PolynomialSpline::PolynomialSpline (const std::string& component_name)
    : Component(-1, component_name)
{
}

PolynomialSpline::~PolynomialSpline ()
{
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
PolynomialSpline::GetJacobian (double t_global, MotionDerivative deriv, int dim) const
{
  assert(dim < n_dim_);

  int id         = GetSegmentID(t_global);
  double t_local = GetLocalTime(t_global);

  return GetJacobianWrtCoeffAtPolynomial(deriv, t_local, id, dim);
}

Jacobian
PolynomialSpline::GetJacobian (double t_global, MotionDerivative deriv) const
{
  Jacobian jac(n_dim_, GetRows());
  for (int dim=0; dim<n_dim_; ++dim)
    jac.row(dim) = GetJacobian(t_global, deriv, dim);

  return jac;
}

JacobianRow
PolynomialSpline::GetJacobianWrtCoeffAtPolynomial (MotionDerivative deriv, double t_local,
                                                   int id, int dim) const
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


EndeffectorSpline::EndeffectorSpline(const std::string& id, bool first_phase_in_contact)
   : PolynomialSpline(id)
{
  first_phase_in_contact_ = first_phase_in_contact;
}

EndeffectorSpline::~EndeffectorSpline ()
{
}

VecBound
EndeffectorSpline::GetBounds () const
{
  VecBound bounds(GetRows());
  std::fill(bounds.begin(), bounds.end(), kNoBound_);

  bool is_contact = first_phase_in_contact_;

  int i = 0;
  for (const auto& p : GetPolynomials()) {
    for (int dim=0; dim<GetNDim(); ++dim)
      for (auto coeff : p->GetCoeffIds()) {

        if(is_contact && (coeff != Polynomial::A)) {
          bounds.at(Index(i,dim,coeff)) = kEqualityBound_;
        }

        // zmp_ this one should depend on x,y -> formulate as constraint
        // that will allow rough terrain.
        if(is_contact && dim==Z) {
          bounds.at(Index(i,dim,coeff)) = kEqualityBound_;
        }
      }

    is_contact = !is_contact; // after contact phase MUST come a swingphase (by definition).
    i++;
  }


  return bounds;
}



ForceSpline::ForceSpline(const std::string& id, bool first_phase_in_contact)
   : PolynomialSpline(id)
{
  first_phase_in_contact_ = first_phase_in_contact;
}

ForceSpline::~ForceSpline ()
{
}

VecBound
ForceSpline::GetBounds () const
{
  const double max_force_ = 20000.0; // [N]

  VecBound bounds(GetRows());
  std::fill(bounds.begin(), bounds.end(), Bound(-max_force_, max_force_));

  // zmp_ use motion param for this

  bool is_contact = first_phase_in_contact_;

  int i = 0;
  for (const auto& p : GetPolynomials()) {
    for (int dim=0; dim<GetNDim(); ++dim)
      for (auto coeff : p->GetCoeffIds()) {

        if(!is_contact) { // can't produce forces during swingphase
          bounds.at(Index(i,dim,coeff)) = kEqualityBound_;
        }

        // unilateral contact forces
        if(is_contact && dim==Z) {
          bounds.at(Index(i,dim,coeff)) = Bound(0.0, max_force_);
        }
      }

    is_contact = !is_contact; // after contact phase MUST come a swingphase (by definition).
    i++;
  }


  return bounds;
}


} /* namespace opt */
} /* namespace xpp */

