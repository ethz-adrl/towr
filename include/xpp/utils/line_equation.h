/**
 @file    line_equation.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 28, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINE_EQUATION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINE_EQUATION_H_

#include <Eigen/Dense>

namespace xpp {
namespace utils {

/** p*x + q*y + r = 0 */
struct LineCoeff2d {
  double p;
  double q;
  double r;
};

// overloading operator<< for more elegant priting of above values
inline std::ostream& operator<<(std::ostream& out, const LineCoeff2d& lc)
{
  out  << "p=" << lc.p << ", q=" << lc.q << ", r=" << lc.r;
  return out;
}


class LineEquation {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static const int kNumLineCoeff = 3;   // p, q, r
  static const int kNumPointCoords = 4; // p0.x, p0.y, p1.x, p1.y
  typedef Eigen::Matrix<double, kNumLineCoeff, kNumPointCoords> JacobianCoeff;
  typedef Eigen::Matrix<double, 1, kNumPointCoords>  JacobianRow;

  typedef Eigen::Vector2d Point;

  LineEquation ();
  LineEquation (const Point& pt0, const Point& pt1);
  virtual ~LineEquation ();

  void SetPoints(const Point& pt0, const Point& pt1);
  LineCoeff2d GetCoeff() const;
  double GetDistanceFromLine(const Point& pt) const;


  JacobianCoeff GetJacobianCoeffWrtPoints() const;


private:
  Point pt0, pt1;
};


} /* namespace utils */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINE_EQUATION_H_ */
