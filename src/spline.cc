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

  std::string s1 = id::endeffectors_motion;
  std::string s2 = id::endeffector_force;
  if (name.substr(0, s1.size()) == s1) // string starts with s
    spline = std::make_shared<HermiteSpline>(opt_vars, name);
  else if (name.substr(0, s2.size()) == s2) // string starts with s
    spline = std::make_shared<HermiteSpline>(opt_vars, name);
  else if (name == id::base_linear)
    spline = std::make_shared<HermiteSpline>(opt_vars, name);
  else if (name == id::base_angular)
    spline = std::make_shared<HermiteSpline>(opt_vars, name);
  else
    assert(false); // this shouldn't happen

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
   for (double d: GetDurations()) {
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
    t_local -= GetDurations().at(id);
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


} /* namespace opt */
} /* namespace xpp */
