/**
 @file    polynomial_fifth_order.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 3, 2016
 @brief   Brief description
 */

#include <xpp/zmp/polynomial_fifth_order.h>
#include <iostream>

namespace xpp {
namespace zmp {

using namespace xpp::utils;

PolynomialFifthOrder::PolynomialFifthOrder()
{
  for (int dim = 0; dim < kDim2d; ++dim)
    for (int coeff = 0; coeff < kCoeffCount; ++coeff)
      spline_coeff_[dim][coeff] = 0.0;
}

PolynomialFifthOrder::~PolynomialFifthOrder ()
{
  // TODO Auto-generated destructor stub
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

} /* namespace zmp */
} /* namespace xpp */
