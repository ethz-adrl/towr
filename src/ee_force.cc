/**
 @file    ee_force.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 15, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/ee_force.h>

namespace xpp {
namespace opt {

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
}

VectorXd
EEForce::GetValues () const
{
  // get all the node values
  int num_nodes = spline_.size()+1;
  VectorXd x(num_nodes*dim_.size());

  int idx=0;
  for (const auto& p : spline_)
    for (auto d : dim_) {
      x(idx)   = p.start_.p_(d);
      x(++idx) = p.end_.p_(d);
    }

  return x;
}

void
EEForce::SetValues (const VectorXd& x)
{
  int idx=0;
  for (auto& p : spline_) {
    for (auto d : dim_) {
      p.start_.p_(d) = x(idx);
      p.end_.p_(d)   = x(++idx);
      p.UpdateCoefficients();
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
  for (int s=0; s<spline_.size(); ++s) {

    if (!is_in_contact_.at(s)) // swing phase
      for (auto d : dim_)
        for (int idx : {i, i+1})
          bounds.at(idx) = kEqualityBound_; // no contact forces allowed

    i += dim_.size();
  }

  return bounds_;
}


double
EEForce::GetForce (double t_global) const
{

}

int
EEForce::Index (double t_global) const
{

}

void
EEForce::AddContactPhase (double T)
{
  PolynomialType p;

  double t_left = T;
  while (t_left > 0.0) {
    double duration = t_left>dt_?  dt_ : t_left;
    p.SetBoundary(duration, StateLin1d(), StateLin1d());
    spline_.push_back(p);
    is_in_contact_.push_back(true);

    for (auto d : dim_)
      for (int i=0; i<2; ++i) // start and end values
        bounds_.push_back(Bound(min_load_, max_load_)); // unilateral force

    t_left -= dt_;
  }
}

void
EEForce::AddSwingPhase (double T)
{
  // one polynomials for entire swing-phase, with value zero
  PolynomialType p;
  p.SetBoundary(T, StateLin1d(), StateLin1d());

  for (auto d : dim_)
    for (int i=0; i<2; ++i) // start and end values
      bounds_.push_back(kEqualityBound_); // zero force

  spline_.push_back(p);
  is_in_contact_.push_back(true);
}

} /* namespace opt */
} /* namespace xpp */
