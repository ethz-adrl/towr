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

EEForce::EEForce (double dt) : Component(-1, "ee_force_single")
{
  dt_ = dt;
}

EEForce::~EEForce ()
{
}

void
EEForce::AddPhase (double T, bool is_contact)
{
  if (is_contact)
    AddContactPhase(T);
  else
    AddSwingPhase(T);

  spline_.SetSegmentsPtr(polynomials_); // update spline here
}

VectorXd
EEForce::GetValues () const
{
  // get all the node values
  int num_nodes = polynomials_.size()+1;
  VectorXd x(num_nodes*dim_.size());

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
  // first assume all can carry load
  bounds.assign(GetRows(),Bound(min_load_, max_load_));

  int i=0;
  for (int s=0; s<polynomials_.size(); ++s) {

    if (!is_in_contact_.at(s)) // swing phase
      for (auto d : dim_)
        for (int idx : {i, i+1})
          bounds.at(idx) = kEqualityBound_; // no contact forces allowed

    i += dim_.size();
  }

  return bounds;
}


double
EEForce::GetForce (double t_global) const
{
  return spline_.GetPoint(t_global).p_(0);
}

int
EEForce::Index (double t_global) const
{
  // zmp_!!! implement this
  int poly_id = spline_.GetSegmentID(t_global);


  assert(false);
}

void
EEForce::AddContactPhase (double T)
{
  auto p = std::make_shared<PolynomialType>();

  double t_left = T;
  while (t_left > 0.0) {
    double duration = t_left>dt_?  dt_ : t_left;
    p->SetBoundary(duration, StateLin1d(), StateLin1d());
    polynomials_.push_back(p);
    is_in_contact_.push_back(true);
    t_left -= dt_;
  }
}

void
EEForce::AddSwingPhase (double T)
{
  // one polynomials for entire swing-phase, with value zero
  auto p = std::make_shared<PolynomialType>();
  p->SetBoundary(T, StateLin1d(), StateLin1d());

  polynomials_.push_back(p);
  is_in_contact_.push_back(true);
}

} /* namespace opt */
} /* namespace xpp */
