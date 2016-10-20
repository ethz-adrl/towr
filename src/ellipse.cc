/**
@file    ellipse.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jul 20, 2016
@brief   Brief description
 */

#include <xpp/opt/ellipse.h>

#include <cmath>

namespace xpp {
namespace opt {

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

double
Ellipse::GetConstant () const
{
  double constant = 0.0;
  constant += std::pow(2*i_rx_m_/x_axis_length_,2);
  constant += std::pow(2*i_ry_m_/y_axis_length_,2);
  constant -= 1;

  return constant;
}

Ellipse::JacobianRow
Ellipse::GetJacobianWrtXY (double i_rx_p, double i_ry_p) const
{
  int X = 0;
  int Y = 1;
  JacobianRow jac;
  jac.at(X) = 8./std::pow(x_axis_length_,2)*(i_rx_p - i_rx_m_);
  jac.at(Y) = 8./std::pow(y_axis_length_,2)*(i_ry_p - i_ry_m_);

  return jac;
}


} /* namespace zmp */
} /* namespace xpp */

