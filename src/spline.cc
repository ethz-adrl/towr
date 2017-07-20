/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/variables/spline.h>

#include <cassert>

#include <xpp/opt/variables/variable_names.h>

#include <xpp/opt/variables/node_values.h>
#include <xpp/opt/variables/coeff_spline.h>

namespace xpp {
namespace opt {

Spline::Ptr
Spline::BuildSpline (const OptVarsPtr& opt_vars,
                     const std::string& name,
                     const VecTimes& poly_durations)
{
  Ptr spline;

  std::string s = id::endeffectors_motion;
  if (name.substr(0, s.size()) == s) // string starts with s
    spline = std::make_shared<HermiteSpline>(opt_vars, name, poly_durations);
  else if (name == id::base_linear)
    spline = std::make_shared<HermiteSpline>(opt_vars, name, poly_durations);
  else if (name == id::base_angular)
    spline = std::make_shared<HermiteSpline>(opt_vars, name, poly_durations);
  else
    spline = std::make_shared<CoeffSpline>(opt_vars, name, poly_durations);

  return spline;
}


Spline::Spline ()
{
}

Spline::~Spline ()
{
}

int
Spline::GetSegmentID(double t_global) const
{
  double eps = 1e-10; // double imprecision

   double t = 0;
   int i=0;
   for (double d: durations_) {
     t += d;

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }
   assert(false); // this should never be reached
}

double
Spline::GetLocalTime(double t_global) const
{
  int id_spline = GetSegmentID(t_global);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= durations_.at(id);
  }

  return t_local;
}

Spline::PPtr
Spline::GetActivePolynomial(double t_global) const
{
  int id = GetSegmentID(t_global);
  return polynomials_.at(id);
}


const StateLinXd
Spline::GetPoint(double t_global) const
{
  double t_local = GetLocalTime(t_global);
  return GetActivePolynomial(t_global)->GetPoint(t_local);
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
