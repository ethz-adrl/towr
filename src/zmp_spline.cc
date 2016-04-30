/**
@file   zmp_spline.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Spline created by the zmp optimizaton and added to SplineContainer.
 */

#include <xpp/zmp/zmp_spline.h>

#include <ros/console.h>
#include <iostream>

namespace xpp {
namespace zmp {


Spline::Spline()
{
  for (int dim = 0; dim < kDim2d; ++dim)
    for (int coeff = 0; coeff < kCoeffCount; ++coeff)
      spline_coeff_[dim][coeff] = 0.0;
}

Spline::Spline(const CoeffValues &coeff_values)
{
  set_spline_coeff(coeff_values);
}


Spline::~Spline()
{
}


Spline::Vec2d Spline::GetState(const PosVelAcc &whichDerivative,
                         const double& _t) const
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
    case kPos:
      ret[dim] =    a*t[5] +    b*t[4] +   c*t[3] +   d*t[2] + e*t[1] + f;
      break;
    case kVel:
      ret[dim] =  5*a*t[4] +  4*b*t[3] + 3*c*t[2] + 2*d*t[1] + e;
      break;
    case kAcc:
      ret[dim] = 20*a*t[3] + 12*b*t[2] + 6*c*t[1] + 2*d;
      break;
    default:
      ROS_ERROR("Spline.GetState: Do you want pos, vel or acc info? returning 0.0");
      ret[dim] = 0.0;
    }
  }
  return ret;
}



void Spline::set_spline_coeff(const CoeffValues &coeff_values)
{
    for (int c = 0; c < kCoeffCount; ++c) {
      spline_coeff_[utils::X][c] = coeff_values.x[c];
      spline_coeff_[utils::Y][c] = coeff_values.y[c];
    }
}

ZmpSpline::ZmpSpline()
    : duration_(0.0),
      id_(0),
      four_leg_supp_(false),
      step_(0)
{
  set_spline_coeff();
}


ZmpSpline::ZmpSpline(unsigned int id, double duration, bool four_leg_supp, int step)
    : id_(id), duration_(duration), four_leg_supp_(four_leg_supp), step_(step)
{
  set_spline_coeff();
}


} // namespace zmp
} // namespace xpp
