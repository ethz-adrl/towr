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





double
PolynomialSpline::GetTotalTime() const
{
  double T = 0.0;
  for (const auto& s: polynomials_) {
    T += s->GetDuration();
  }
  return T;
}

const StateLinXd
PolynomialSpline::GetPoint(double t_global) const
{
  int idx        = GetSegmentID(t_global);
  double t_local = GetLocalTime(t_global);

  return GetPoint(idx, t_local);
}

double
PolynomialSpline::GetLocalTime(double t_global) const
{
  int id_spline = GetSegmentID(t_global);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= polynomials_.at(id)->GetDuration();
  }

  return t_local;//-eps_; // just to never get value greater than true duration due to rounding errors
}

int
PolynomialSpline::GetSegmentID(double t_global) const
{
  double eps = 1e-10; // double imprecision
  assert(t_global<=GetTotalTime()+eps); // machine precision

   double t = 0;
   int i=0;
   for (const auto& s: polynomials_) {
     t += s->GetDuration();

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }
   assert(false); // this should never be reached
}

const StateLinXd
PolynomialSpline::GetPoint (int id, double t_local) const
{
  return polynomials_.at(id)->GetPoint(t_local);
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
  return GetJacobian(t_global, deriv).row(dim);
}

Jacobian
PolynomialSpline::GetJacobian (double t_global, MotionDerivative deriv) const
{
  int id         = GetSegmentID(t_global);
  double t_local = GetLocalTime(t_global);

  return GetJacobian(id, t_local, deriv);
}

Jacobian
PolynomialSpline::GetJacobian (int id, double t_local, MotionDerivative dxdt) const
{
  Jacobian jac(n_dim_, GetRows());
  for (int dim=0; dim<n_dim_; ++dim)
    jac.row(dim) = GetJacobianWrtCoeffAtPolynomial(dxdt, t_local, id, dim);

  return jac;
}

JacobianRow
PolynomialSpline::GetJacobianWrtCoeffAtPolynomial (MotionDerivative deriv,
                                                   double t_local,
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



ForceSpline::ForceSpline(const std::string& id, bool first_phase_in_contact, double max_force)
   : PolynomialSpline(id)
{
  first_phase_in_contact_ = first_phase_in_contact;
  max_force_ = max_force;
}

ForceSpline::~ForceSpline ()
{
}

VecBound
ForceSpline::GetBounds () const
{
  VecBound bounds(GetRows());
  std::fill(bounds.begin(), bounds.end(), Bound(-max_force_, max_force_));

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

    i++;

    if (i%n_polys_per_phase_ == 0)
      is_contact = !is_contact; // after contact phase MUST come a swingphase (by definition).

  }


  return bounds;
}


} /* namespace opt */
} /* namespace xpp */
