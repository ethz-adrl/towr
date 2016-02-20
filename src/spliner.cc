/**
@file   spliner.h
@author Alexander Wayne Winkler
@date   29.07.2014

@brief  A virtual class spliner with ready to use derived spliners

Spliners ready to use:
        - Linear Spliner
        - Cubic Spliner
        - Quintic Spliner
*/

#include <xpp/utils/spliner.h>

namespace xpp {
namespace utils {

/**
 * The spliner always calculates the splines in the same way, but if the
 * spline coefficients are zero (as set by @ref Spliner()), the higher-order
 * terms have no effect
 */
bool Spliner::GetPoint(const double dt, Point& out) const
{
  // sanity checks
  if (dt < 0)
    throw std::runtime_error("spliner.cc called with dt<0");

  double dt1 = (dt > duration) ? duration : dt;
  double dt2 = dt1 * dt1;
  double dt3 = dt1 * dt2;
  double dt4 = dt1 * dt3;
  double dt5 = dt1 * dt4;

  /** Different spline type are generated, by setting the coefficients > order of spline to zero  */
  out.x   = c[0] + c[1]*dt1 +   c[2]*dt2 +   c[3]*dt3 +    c[4]*dt4 +    c[5]*dt5;
  out.xd  =        c[1]     + 2*c[2]*dt1 + 3*c[3]*dt2 +  4*c[4]*dt3 +  5*c[5]*dt4;
  out.xdd =                   2*c[2]     + 6*c[3]*dt1 + 12*c[4]*dt2 + 20*c[5]*dt3;

  return true;
}


void LinearSpliner::CalcSplineCoeff(double T, const Point& start, const Point& end)
{
  c[0] = start.x;
  c[1] = (end.x - start.x) / T;

  c[2] = c[3] = c[4] = c[5] = 0.0;
}


void CubicSpliner::CalcSplineCoeff(double T, const Point& start, const Point& end)
{
  double T1 = T;
  double T2 = T1 * T1;
  double T3 = T1 * T2;

  c[0] = start.x;
  c[1] = start.xd;
  c[2] = - ((3 * c[0]) - (3 * end.x) + (2 * T1 * c[1]) + (T1 * end.xd)) / T2;
  c[3] =   ((2 * c[0]) - (2 * end.x) +      T1 * (c[1] +       end.xd ))/ T3;

  c[4] = c[5] = 0.0;
}


void QuinticSpliner::CalcSplineCoeff(double T, const Point& start, const Point& end)
{
  double T1 = T;
  double T2 = T1 * T1;
  double T3 = T1 * T2;
  double T4 = T1 * T3;
  double T5 = T1 * T4;

  c[0] = start.x;
  c[1] = start.xd;
  c[2] = start.xdd / 2.;
  c[3] =  (-20*start.x + 20*end.x + T1*(-3*start.xdd*T1 +   end.xdd*T1 - 12* start.xd -  8*end.xd))  / (2*T3);
  c[4] =  ( 30*start.x - 30*end.x + T1*( 3*start.xdd*T1 - 2*end.xdd*T1 + 16* start.xd + 14*end.xd))  / (2*T4);
  c[5] = -( 12*start.x - 12*end.x + T1*(   start.xdd*T1 -   end.xdd*T1 +  6*(start.xd +    end.xd))) / (2*T5);
}

} // namespace utils
} // namespace xpp
