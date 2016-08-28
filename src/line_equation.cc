/**
 @file    line_equation.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 28, 2016
 @brief   Brief description
 */

#include <xpp/utils/line_equation.h>

namespace xpp {
namespace utils {

LineEquation::~LineEquation ()
{
  // TODO Auto-generated destructor stub
}

LineEquation::LineEquation (const Vec2d& _pt0, const Vec2d& _pt1)
{
  SetPoints(_pt0, _pt1);
}

void
LineEquation::SetPoints (const Vec2d& _pt0, const Vec2d& _pt1)
{
  pt0 = _pt0;
  pt1 = _pt1;
}

LineEquation::LineEquation ()
{
}

// from http://math.stackexchange.com/questions/1076292/obtain-coefficients-of-a-line-from-2-points
LineCoeff2d
LineEquation::GetCoeff(bool normalize) const {

  LineCoeff2d ret;
  ret.p = pt0.y() - pt1.y();
  ret.q = pt1.x() - pt0.x();
//    ret.r = -ret.p * pt0.x() - ret.q * pt0.y();
  ret.r = pt0.x()*pt1.y() - pt1.x()*pt0.y();

  // normalize the equation in order to intuitively use stability margins
  if (normalize) {
    double norm = hypot(ret.p, ret.q);
    ret.p /= norm;
    ret.q /= norm;
    ret.r /= norm;
  }

  return ret;
}

} /* namespace utils */
} /* namespace xpp */

