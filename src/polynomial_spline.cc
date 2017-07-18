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

PolynomialSpline::PolynomialSpline ()
{
}

PolynomialSpline::~PolynomialSpline ()
{
}


PolynomialSpline::PtrS
PolynomialSpline::BuildSpline(const OptVarsPtr& opt_vars,
                              const std::string& spline_base_id,
                              const VecTimes& poly_durations)
{
  auto spline_ = std::make_shared<PolynomialSpline>();
  spline_->SetDurations(poly_durations);
  for (int i=0; i<poly_durations.size(); ++i) {
    auto p = std::dynamic_pointer_cast<Polynomial>(opt_vars->GetComponent(spline_base_id+std::to_string(i)));
    spline_->AddPolynomial(p);
  }

  return spline_;
}

double
PolynomialSpline::GetLocalTime(double t_global, const VecTimes& durations)
{
  int id_spline = GetSegmentID(t_global, durations);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= durations.at(id);
  }

  return t_local;
}

int
PolynomialSpline::GetSegmentID(double t_global, const VecTimes& durations)
{
  double eps = 1e-10; // double imprecision

   double t = 0;
   int i=0;
   for (double d: durations) {
     t += d;

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }
   assert(false); // this should never be reached
}

const StateLinXd
PolynomialSpline::GetPoint(double t_global) const
{
  int idx        = GetSegmentID(t_global, durations_);
  double t_local = GetLocalTime(t_global, durations_);

  return polynomials_.at(idx)->GetPoint(t_local);
}

Jacobian
PolynomialSpline::GetJacobian (double t_global, MotionDerivative deriv) const
{
  int id         = GetSegmentID(t_global,durations_);
  double t_local = GetLocalTime(t_global,durations_);

  return polynomials_.at(id)->GetJacobian(t_local, deriv);
}

void
PolynomialSpline::AddPolynomial(const PolynomialPtr& poly)
{
  polynomials_.push_back(poly);
}

void
PolynomialSpline::SetDurations(const VecTimes& durations)
{
  durations_ = durations;
}

bool
PolynomialSpline::PolynomialActive(const std::string& poly_name, double t_global)
{
  int id = GetSegmentID(t_global, durations_);
  return polynomials_.at(id)->GetName() == poly_name;
}


PolynomialSpline::PolynomialPtr
PolynomialSpline::GetActivePolynomial(double t_global) const
{
  int id = GetSegmentID(t_global, durations_);
  return polynomials_.at(id);
}






//VecBound
//EndeffectorSpline::GetBounds () const
//{
//  VecBound bounds(GetRows());
//  std::fill(bounds.begin(), bounds.end(), kNoBound_);
//
//  bool is_contact = first_phase_in_contact_;
//
//  int i = 0;
//  for (const auto& p : GetPolynomials()) {
//    for (int dim=0; dim<GetNDim(); ++dim)
//      for (auto coeff : p.GetCoeffIds()) {
//
//        if(is_contact && (coeff != Polynomial::A)) {
//          bounds.at(Index(i,dim,coeff)) = kEqualityBound_;
//        }
//
//        // zmp_ this one should depend on x,y -> formulate as constraint
//        // that will allow rough terrain.
//        if(is_contact && dim==Z) {
//          bounds.at(Index(i,dim,coeff)) = kEqualityBound_;
//        }
//      }
//
//    is_contact = !is_contact; // after contact phase MUST come a swingphase (by definition).
//    i++;
//  }
//
//
//  return bounds;
//}


//ForceSpline::ForceSpline(const std::string& id, bool first_phase_in_contact, double max_force)
//   : PolynomialSpline(id)
//{
//  first_phase_in_contact_ = first_phase_in_contact;
//  max_force_ = max_force;
//}
//
//ForceSpline::~ForceSpline ()
//{
//}
//
//VecBound
//ForceSpline::GetBounds () const
//{
//  VecBound bounds(GetRows());
//  std::fill(bounds.begin(), bounds.end(), Bound(-max_force_, max_force_));
//
//  bool is_contact = first_phase_in_contact_;
//
//  int i = 0;
//  for (const auto& p : GetPolynomials()) {
//
//
//    for (int dim=0; dim<GetNDim(); ++dim)
//      for (auto coeff : p.GetCoeffIds()) {
//
//        if(!is_contact) { // can't produce forces during swingphase
//          bounds.at(Index(i,dim,coeff)) = kEqualityBound_;
//        }
//
//        // unilateral contact forces
//        if(is_contact && dim==Z) {
//          bounds.at(Index(i,dim,coeff)) = Bound(0.0, max_force_);
//        }
//      }
//
//    i++;
//
//    if (i%n_polys_per_phase_ == 0)
//      is_contact = !is_contact; // after contact phase MUST come a swingphase (by definition).
//
//  }
//
//
//  return bounds;
//}


} /* namespace opt */
} /* namespace xpp */
