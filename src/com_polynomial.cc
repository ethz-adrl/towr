/**
@file   com_polynomial.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2015
@brief  Defines CoeffValues, PolynomialFifthOrder and ComPolynomial
 */

#include <iostream>
#include <xpp/zmp/com_polynomial.h>

namespace xpp {
namespace zmp {

using namespace xpp::utils::coords_wrapper;

PolynomialFifthOrder::PolynomialFifthOrder()
{
  for (int dim = 0; dim < kDim2d; ++dim)
    for (int coeff = 0; coeff < kCoeffCount; ++coeff)
      spline_coeff_[dim][coeff] = 0.0;
}

PolynomialFifthOrder::PolynomialFifthOrder(const CoeffValues &coeff_values)
{
  SetSplineCoefficients(coeff_values);
}


PolynomialFifthOrder::Vec2d
PolynomialFifthOrder::GetState(PosVelAcc whichDerivative, double _t) const
{
  // caching the exponential times
  double t[6];
  t[0] = 1;
  for (int i = 1; i < 6; i++) { t[i] = t[i-1] * _t; }

  Vec2d ret;

  for (int dim = 0; dim < kDim2d; ++dim) {

    /// some aliases for easier usage
    const double &a = spline_coeff_[dim][0];
    const double &b = spline_coeff_[dim][1];
    const double &c = spline_coeff_[dim][2];
    const double &d = spline_coeff_[dim][3];
    const double &e = spline_coeff_[dim][4];
    const double &f = spline_coeff_[dim][5];

    switch (whichDerivative) {
    case xpp::utils::kPos:
      ret[dim] =    a*t[5] +    b*t[4] +   c*t[3] +   d*t[2] + e*t[1] + f;
      break;
    case xpp::utils::kVel:
      ret[dim] =  5*a*t[4] +  4*b*t[3] + 3*c*t[2] + 2*d*t[1] + e;
      break;
    case xpp::utils::kAcc:
      ret[dim] = 20*a*t[3] + 12*b*t[2] + 6*c*t[1] + 2*d;
      break;
    case xpp::utils::kJerk:
      ret[dim] = 60*a*t[2] + 24*b*t[1] + 6*c;
      break;
    default:
      std::cerr << "Spline.GetState: Do you want pos, vel or acc info? returning 0.0";
      ret[dim] = 0.0;
    }
  }
  return ret;
}

void PolynomialFifthOrder::SetSplineCoefficients(const CoeffValues &coeff_values)
{
    for (int c = 0; c < kCoeffCount; ++c) {
      spline_coeff_[utils::X][c] = coeff_values.x[c];
      spline_coeff_[utils::Y][c] = coeff_values.y[c];
    }
}

double
PolynomialFifthOrder::GetCoefficient (int dim, SplineCoeff coeff) const
{
  return spline_coeff_[dim][coeff];
}

ComPolynomial::ComPolynomial() : id_(0), duration_(0.0)
{
  SetSplineCoefficients();
}

ComPolynomial::ComPolynomial(uint id, double duration) : id_(id), duration_(duration)
{
  SetSplineCoefficients();
}

double
ComPolynomial::GetTotalTime(const VecPolynomials& splines)
{
  double T = 0.0;
  for (const ComPolynomial& s: splines)
    T += s.GetDuration();
  return T;
}

double
ComPolynomial::GetLocalTime(double t_global, const VecPolynomials& splines)
{
  int id_spline = GetPolynomialID(t_global,splines);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= splines.at(id).GetDuration();
  }

  return t_local;//-eps_; // just to never get value greater than true duration due to rounding errors
}

ComPolynomial::Point2d
ComPolynomial::GetCOM(double t_global, const VecPolynomials& splines)
{
  int id = GetPolynomialID(t_global,splines);
  double t_local = GetLocalTime(t_global, splines);

  return GetCOGxyAtPolynomial(id, t_local, splines);
}

ComPolynomial::Point2d
ComPolynomial::GetCOGxyAtPolynomial (int id, double t_local, const VecPolynomials& splines)
{
  Point2d cog_xy;
  cog_xy.p = splines[id].GetState(kPos, t_local);
  cog_xy.v = splines[id].GetState(kVel, t_local);
  cog_xy.a = splines[id].GetState(kAcc, t_local);
  cog_xy.j = splines[id].GetState(kJerk, t_local);

  return cog_xy;
}

int
ComPolynomial::GetPolynomialID(double t_global, const VecPolynomials& splines)
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

std::ostream&
operator<<(std::ostream& out, const ComPolynomial& p)
{
  out << "id: " << p.id_
      << "\t duration: " << p.duration_;
  return out;
}

} // namespace zmp
} // namespace xpp

