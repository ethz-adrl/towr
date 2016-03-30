/**
@file    point2d_manipulations.h
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Utilites for creating lines with points, calculating distances,...
 */

#ifndef _XPP_UTILS_POINT2D_MANIPULATOR_H_
#define _XPP_UTILS_POINT2D_MANIPULATOR_H_

#include "geometric_structs.h"
#include <Eigen/StdVector>

#include <array>
#include <algorithm> // std::sort
#include <cmath>
#include <iostream>


namespace xpp {
namespace utils {

/**
@brief Some utilities for manipulating 2 dimensional points, distances, lines.
 */
struct Point2dManip {

  typedef std::vector<Vec2d, Eigen::aligned_allocator<Vec2d> > StdVectorEig2d;


  // from http://math.stackexchange.com/questions/1076292/obtain-coefficients-of-a-line-from-2-points
  static LineCoeff2d LineCoeff(const Vec2d& pt0, const Vec2d& pt1, bool normalize = true) {

    LineCoeff2d ret;
    ret.p = pt0.y() - pt1.y();
    ret.q = pt1.x() - pt0.x();
    ret.r = -ret.p * pt0.x() - ret.q * pt0.y(); //pt0.x()*pt1.y() - pt1.x()*pt0.y();

    // normalize the equation in order to intuitively use stability margins
    if (normalize) {
      double norm = hypot(ret.p, ret.q);
      ret.p /= norm;
      ret.q /= norm;
      ret.r /= norm;
    }

    return ret;
  }


  static double Distance(Vec2d pt1, Vec2d pt2);

  /// \brief  Checks if p2 is on the right side of line from p0 to p1
  /// \return >0 for P2 right of the line from P0 to P1
  ///         =0 for P2 on the line
  ///         <0 for P2 left of the line
  static double Point2isRightOfLine(const Vec2d& p0, const Vec2d p1, const Vec2d& p2)
  {
    return (p2(X) - p0(X)) * (p1(Y) - p0(Y))
         - (p1(X) - p0(X)) * (p2(Y) - p0(Y));
  }



  static bool SortByXThenY(const Vec2d& lhs, const Vec2d& rhs)
  {
    if (lhs(X) != rhs(X))
      return lhs(X) < rhs(X);
    else
      return lhs(Y) < rhs(Y);
  }

  /**
  \brief Performs a counter clockwise radial sort of the input points
         starting with the bottom left point
  @param[out] p Unsorted points or footholds.
  Uses the slow n^2 sort algorithm, shouldn't matter for sorting 4 points :)
  Fails when 3 points are on same line and one could be removed
   */
//  template<std::size_t N >
  static void CounterClockwiseSort(StdVectorEig2d& p)
  {
    // make sure the first point is the one with lowest (x,y) value
    std::sort(p.begin(), p.end(), SortByXThenY);

    // sort counterclockwise
    for (size_t i = 1; i < p.size() - 1; i++) {
      for (size_t j = i + 1; j < p.size(); j++) {
        if (Point2isRightOfLine(p[0], p[i], p[j])  > 0.0) {
          Vec2d tmp = p[i];
          p[i] = p[j];
          p[j] = tmp;
        }
      }
    }
  }

};


} // namespace utils
} // namespace xpp

#endif // _XPP_UTILS_POINT2D_MANIPULATOR_H_
