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

using PolynomialUsed = QuarticPolynomial;

PolynomialSpline::PolynomialSpline (const std::string& component_name)
    : Component(-1, component_name)
{
}

PolynomialSpline::~PolynomialSpline ()
{
}

void
PolynomialSpline::Init (double t_global, double dt, const VectorXd& initial_value)
{
  // initialize at com position with zero velocity & acceleration
  n_dim_ = initial_value.rows();
  State initial_state(n_dim_);
  initial_state.p_ = initial_value;

  double t_left = t_global;
  while (t_left > 0.0) {
    double duration = t_left>dt?  dt : t_left;
    auto p = std::make_shared<PolynomialUsed>();
    p->SetBoundary(duration, initial_state, initial_state);
    polynomials_.push_back(p);
    t_left -= dt;
  }

  SetSegmentsPtr(polynomials_);

  int n_polys = polynomials_.size();
  SetRows(n_polys*GetFreeCoeffPerPoly()*n_dim_);
}

void
PolynomialSpline::Init (std::vector<double> T_polys, const VectorXd& initial_value)
{
  // initialize at com position with zero velocity & acceleration
  n_dim_ = initial_value.rows();
  State initial_state(n_dim_);
  initial_state.p_ = initial_value;

  for (double duration : T_polys) {
    auto p = std::make_shared<PolynomialUsed>();
    p->SetBoundary(duration, initial_state, initial_state);
    polynomials_.push_back(p);
  }

  // DRY with above init
  SetSegmentsPtr(polynomials_);
  int n_polys = polynomials_.size();
  SetRows(n_polys*GetFreeCoeffPerPoly()*n_dim_);
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
      for (auto coeff : p->GetCoeffIds())
        if(is_contact && (coeff != Polynomial::A || dim==Z)) {
          bounds.at(Index(i,dim,coeff)) = kEqualityBound_; // allow slight movement for numerics
          // zmp_ remove
//          std::cout << "setting bounds zero for p: " << i << ",dim: " << dim << ", coeff: " << coeff << std::endl;
        }

    is_contact = !is_contact;
    i++;
  }


  return bounds;
}


} /* namespace opt */
} /* namespace xpp */

