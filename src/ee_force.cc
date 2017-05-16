/**
 @file    ee_force.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 15, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/ee_force.h>

namespace xpp {
namespace opt {

using PolynomialType = ConstantPolynomial;

EEForce::EEForce () : Component(-1, "ee_force_single")
{
}

EEForce::~EEForce ()
{
}

void
EEForce::AddPhase (double T, double dt, bool is_contact)
{
  if (is_contact)
    AddContactPhase(T, dt);
  else
    AddSwingPhase(T);

  spline_.SetSegmentsPtr(polynomials_); // update spline here
  int num_nodes = polynomials_.size()+1;
  SetRows(num_nodes*dim_.size());

  // initial forces should be different from zero
  SetValues(min_load_*VectorXd::Ones(GetRows()));
}

VectorXd
EEForce::GetValues () const
{
  VectorXd x(GetRows());

  int idx=0;
  for (const auto& p : polynomials_)
    for (auto d : dim_) {
      x(idx)   = p->start_.p_(d);
      x(++idx) = p->end_.p_(d);
    }

  return x;
}

void
EEForce::SetValues (const VectorXd& x)
{
  int idx=0;
  for (auto& p : polynomials_) {
    for (auto d : dim_) {
      p->start_.p_(d) = x(idx);
      p->end_.p_(d)   = x(++idx);
      p->UpdateCoefficients();
    }
  }
}

VecBound
EEForce::GetBounds () const
{
  VecBound bounds;
  // first treat all legs as if they were in contact
  bounds.assign(GetRows(),Bound(min_load_, max_load_));

  int i=0;
  for (int s=0; s<polynomials_.size(); ++s) {

    if (!is_in_contact_.at(s)) // swing phase
      for (auto d : dim_) {
        // forbid contact force at start and end of node
        bounds.at(i) = Bound(0.1, 0.2); // [N]
        // zmp_ for now only at start of node b/c using zero-order poly
//        bounds.at(i+1) = Bound(1.0, 2.0);
      }

    i += dim_.size();
  }

  return bounds;
}

double
EEForce::GetForce (double t_global) const
{
  double force_z = spline_.GetPoint(t_global).p_(0);
  return force_z;
}

int
EEForce::Index (double t_global) const
{
  // this must be adapted/extended when changing the polynomial type
  int poly_id = spline_.GetSegmentID(t_global);
  return poly_id*dim_.size() /* + dim */ ; // add this for multiple dimensions
}

void
EEForce::AddContactPhase (double T, double dt)
{

  double t_left = T;
  while (t_left > 0.0) {
    double duration = t_left>dt?  dt : t_left;
    polynomials_.push_back(MakePoly(duration));
    is_in_contact_.push_back(true);
    t_left -= dt;
  }
}

void
EEForce::AddSwingPhase (double T)
{
  // one polynomials for entire swing-phase, with value zero
  polynomials_.push_back(MakePoly(T));
  is_in_contact_.push_back(false);
}

EEForce::PolyPtr
EEForce::MakePoly (double T) const
{
  // initialize with nonzero values, to avoid CoP exception
  auto p = std::make_shared<PolynomialType>();
  p->SetBoundary(T, StateLin1d(), StateLin1d());
  return p;
}

} /* namespace opt */
} /* namespace xpp */
