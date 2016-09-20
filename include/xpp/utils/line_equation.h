/**
 @file    line_equation.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 28, 2016
 @brief   Declares LineEquation and LineCoeff
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

inline std::ostream& operator<<(std::ostream& out, const LineCoeff2d& lc)
{
  out  << "p=" << lc.p << ", q=" << lc.q << ", r=" << lc.r;
  return out;
}

/** Represents a line defined by two points.
  *
  * This class is responsible for encapsulating all calculations related to
  * the construction of a line, distances of points to lines as well as
  * Jacobians of line related values w.r.t. the line defining points pt0, pt1.
  */
class LineEquation {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Point = Eigen::Vector2d;
  static const int kNumLineCoeff = 3;   // p, q, r
  static const int kNumPointCoords = 4; // p0.x, p0.y, p1.x, p1.y
  using JacobianRow = Eigen::Matrix<double, 1, kNumPointCoords>;
  using JacobianCoeff = Eigen::Matrix<double, kNumLineCoeff, kNumPointCoords>;

  LineEquation ();
  LineEquation (const Point& pt0, const Point& pt1);
  virtual ~LineEquation ();

  /** Set two points in the x-y plane through which the line will travel
    */
  void SetPoints(const Point& pt0, const Point& pt1);

  /** Calculates the line coefficients (p,q,r) to pass through the defined point.
    *
    * A line is defined as p*x + q*y + r = 0
    */
  LineCoeff2d GetCoeff() const;

  /** Calculates the Jacobian of the line coefficients (p,q,r) with respect to
    * position of the two defining points.
    *
    * The 3 rows of this Jacobian correspond to the p,q,r coefficient.
    * The 4 columns correspond to the position of the defining points (p0.x, p0.y, p1.x, p1.y)
    */
  JacobianCoeff GetJacobianLineCoeffWrtPoints() const;

  /** Calculates the shortest distance of point pt to the line defined by pt0 and pt1
    */
  double GetDistanceFromLine(const Point& pt) const;

  /** Calculates the Jacobian of the distance function of point pt with respect to
    * the line defined by pt0 and pt1.
    *
    * The 1 row corresponds to the distance of pt to the line
    * The 4 columns correspond to the position of the defining points (p0.x, p0.y, p1.x, p1.y)
    */
  JacobianRow GetJacobianDistanceWrtPoints(const Point& pt) const;

private:
  Point _0, _1;
};


} /* namespace utils */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINE_EQUATION_H_ */
