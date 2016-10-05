/**
@file   com_polynomial.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2015
@brief  Defines CoeffValues, PolynomialFifthOrder and ComPolynomial
 */

#include <iostream>
#include <xpp/utils/polynomial_helpers.h>

namespace xpp {
namespace utils {

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

BaseLin2d
ComPolynomialHelpers::GetCOM(double t_global, const VecPolynomials& splines)
{
  int id = GetPolynomialID(t_global,splines);
  double t_local = GetLocalTime(t_global, splines);

  return GetCOGxyAtPolynomial(id, t_local, splines);
}

BaseLin2d
ComPolynomialHelpers::GetCOGxyAtPolynomial (int id, double t_local, const VecPolynomials& splines)
{
  BaseLin2d cog_xy;
  using namespace xpp::utils;
  cog_xy.p = splines[id].GetState(kPos, t_local);
  cog_xy.v = splines[id].GetState(kVel, t_local);
  cog_xy.a = splines[id].GetState(kAcc, t_local);
  cog_xy.j = splines[id].GetState(kJerk, t_local);

  return cog_xy;
}

int
ComPolynomialHelpers::GetPolynomialID(double t_global, const VecPolynomials& splines)
{
  double eps = 1e-10; // double imprecision
  assert(t_global<=GetTotalTime(splines)+eps); // machine precision

   double t = 0;
   for (const ComPolynomial& s: splines) {
     t += s.GetDuration();

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return s.GetId();
   }
   assert(false); // this should never be reached
}

} // namespace utils
} // namespace xpp

