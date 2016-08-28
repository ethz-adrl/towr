/**
 @file    line_equation.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 28, 2016
 @brief   Defines LineEquation
 */

#include <xpp/utils/line_equation.h>

namespace xpp {
namespace utils {

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
  pt0 = _pt0;
  pt1 = _pt1;
}

LineEquation::LineEquation ()
{
}

// from http://math.stackexchange.com/questions/1076292/obtain-coefficients-of-a-line-from-2-points
LineCoeff2d
LineEquation::GetCoeff() const
{
  LineCoeff2d ret;
  ret.p = pt0.y() - pt1.y();
  ret.q = pt1.x() - pt0.x();
  //    ret.r = -ret.p * pt0.x() - ret.q * pt0.y();
  ret.r = pt0.x()*pt1.y() - pt1.x()*pt0.y();

  // normalize so distance t line function is correct, otherwise
  // point to line depends on how close the two line defining points where apart
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
  double a =  pt0.x() - pt1.x();
  double b =  pt0.y() - pt1.y();
  double c = -pt1.x()*pt1.x()* + pt0.x()*pt1.x() - pt1.y()*pt1.y() + pt0.y()*pt1.y();
  double d = -pt0.x()*pt0.x()* + pt0.x()*pt1.x() - pt0.y()*pt0.y() + pt0.y()*pt1.y();

  JacobianCoeff jac;
  jac.row(0) <<  -a*b,   a*a,  a*b,  -a*a; // coefficient p
  jac.row(1) <<  -b*b,   a*b,  b*b,  -a*b; // coefficient q
  jac.row(2) <<   b*c,  -a*c,  b*d,  -a*d; // coefficient r

  double e =  std::pow(a*a + b*b, 1.5);
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

} /* namespace utils */
} /* namespace xpp */

