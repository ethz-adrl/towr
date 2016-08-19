/**
@file    ellipse.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jul 20, 2016
@brief   Brief description
 */

#include <xpp/zmp/ellipse.h>
#include <cmath>

namespace xpp {
namespace zmp {

Ellipse::Ellipse ()
{
}

Ellipse::Ellipse (double x_axis_length, double y_axis_length, double i_rx_m, double i_ry_m)
{
  x_axis_length_ = x_axis_length;
  y_axis_length_ = y_axis_length;
  i_rx_m_        = i_rx_m;
  i_ry_m_        = i_ry_m;
}

// assumes ellipse axis are aligned with x-y axis of coordinate system
double Ellipse::DistanceToEdge(double i_rx_p, double i_ry_p) const
{
  // distance from point p to midpoint of ellipse m
  double i_rx_mp = i_rx_p - i_rx_m_;
  double i_ry_mp = i_ry_p - i_ry_m_;

  double a = x_axis_length_/2.0;
  double b = y_axis_length_/2.0;

  double ret = 0;
  ret += std::pow(i_rx_mp/a,2);
  ret += std::pow(i_ry_mp/b,2);
  ret -= 1.0;

  return ret;
}

} /* namespace zmp */
} /* namespace xpp */

