/**
 @file    line_equation.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 28, 2016
 @brief   Defines LineEquation
 */

#include <xpp/opt/line_equation.h>

namespace xpp {
namespace opt {

LineEquation::~LineEquation ()
{
  // TODO Auto-generated destructor stub
}

LineEquation::LineEquation (const Point& _pt0, const Point& _pt1)
{
  SetPoints(_pt0, _pt1);
}

void
LineEquation::SetPoints (const Point& _pt0, const Point& _pt1)
{
  _0 = _pt0;
  _1 = _pt1;
}

LineEquation::LineEquation ()
{
}

// from http://math.stackexchange.com/questions/1076292/obtain-coefficients-of-a-line-from-2-points
LineCoeff2d
LineEquation::GetCoeff() const
{
  LineCoeff2d ret;
  ret.p = _0.y() - _1.y();
  ret.q = _1.x() - _0.x();
  ret.r = _0.x()*_1.y() - _1.x()*_0.y();
  //    ret.r = -ret.p * pt0.x() - ret.q * pt0.y();

  // normalize so distance to line function is correct, otherwise
  // distance to line depends on how close the two line defining points where apart
  double norm = hypot(ret.p, ret.q);
  ret.p /= norm;
  ret.q /= norm;
  ret.r /= norm;

  return ret;
}

LineEquation::JacobianCoeff
LineEquation::GetJacobianLineCoeffWrtPoints () const
{
  // as calculated in latex document
  double a =  _0.x() - _1.x();
  double b =  _0.y() - _1.y();
  double c = -_1.x()*_1.x() + _0.x()*_1.x() - _1.y()*_1.y() + _0.y()*_1.y();
  double d = -_0.x()*_0.x() + _0.x()*_1.x() - _0.y()*_0.y() + _0.y()*_1.y();

  double aa = a*a;
  double ab = a*b;
  double bb = b*b;

  JacobianCoeff jac;
  jac.row(0) <<   -ab,    aa,   ab,   -aa; // coefficient p
  jac.row(1) <<   -bb,    ab,   bb,   -ab; // coefficient q
  jac.row(2) <<   b*c,  -a*c,  b*d,  -a*d; // coefficient r

  double e =  std::pow(aa + bb, 1.5);
  jac /= e;

  return jac;
}

double
LineEquation::GetDistanceFromLine (const Point& pt) const
{
  LineCoeff2d l = GetCoeff();
  Eigen::Vector3d coeff(l.p, l.q, l.r);
  Eigen::Vector3d point(pt.x(), pt.y(), 1.0);

  return point.dot(coeff);
}

LineEquation::JacobianRow
LineEquation::GetJacobianDistanceWrtPoints (const Point& pt) const
{
  Eigen::Vector3d point(pt.x(), pt.y(), 1.0);
  JacobianCoeff jac_coeff = GetJacobianLineCoeffWrtPoints();

  return point.transpose()*jac_coeff;
}

} /* namespace opt */
} /* namespace xpp */

