/**
@file   com_polynomial_helpers.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2015
@brief  Defines CoeffValues, PolynomialFifthOrder and ComPolynomial
 */

#include <xpp/opt/com_polynomial_helpers.h>

#include <cassert>
#include <Eigen/Dense>

#include <xpp/cartesian_declarations.h>

#include <xpp/opt/polynomial.h>

namespace xpp {
namespace opt {

double
ComPolynomialHelpers::GetTotalTime(const VecPolynomials& splines)
{
  double T = 0.0;
  for (const ComPolynomial& s: splines)
    T += s.GetDuration();
  return T;
}

double
ComPolynomialHelpers::GetLocalTime(double t_global, const VecPolynomials& splines)
{
  int id_spline = GetPolynomialID(t_global,splines);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= splines.at(id).GetDuration();
  }

  return t_local;//-eps_; // just to never get value greater than true duration due to rounding errors
}

StateLin2d
ComPolynomialHelpers::GetCOM(double t_global, const VecPolynomials& splines)
{
  int idx        = GetPolynomialID(t_global,splines);
  double t_local = GetLocalTime(t_global, splines);

  return GetCOGxyAtPolynomial(idx, t_local, splines);
}

StateLin2d
ComPolynomialHelpers::GetCOGxyAtPolynomial (int idx, double t_local, const VecPolynomials& splines)
{
  StateLin2d cog_xy;
  cog_xy.p = splines[idx].GetState(kPos, t_local);
  cog_xy.v = splines[idx].GetState(kVel, t_local);
  cog_xy.a = splines[idx].GetState(kAcc, t_local);
  cog_xy.j = splines[idx].GetState(kJerk, t_local);

  return cog_xy;
}

int
ComPolynomialHelpers::GetPolynomialID(double t_global, const VecPolynomials& splines)
{
  double eps = 1e-10; // double imprecision
  assert(t_global<=GetTotalTime(splines)+eps); // machine precision

   double t = 0;
   int i=0;
   for (const ComPolynomial& s: splines) {
     t += s.GetDuration();

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }
   assert(false); // this should never be reached
}

} // namespace opt
} // namespace xpp

