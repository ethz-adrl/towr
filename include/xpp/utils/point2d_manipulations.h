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

  //  Checks if B is on the right side of line OA
  static bool BisRightOfOA(const Vec2d& O, const Vec2d A, const Vec2d& B)
  {
    return (Cross(O, A, B) <= 0.0);
  }

  // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
  // Returns a positive value, if OAB makes a counter-clockwise turn,
  // negative for clockwise turn, and zero if the points are collinear.
  static double Cross(const Vec2d &O, const Vec2d &A, const Vec2d &B)
  {
    return (A.x() - O.x()) * (B.y() - O.y())
         - (A.y() - O.y()) * (B.x() - O.x());
  }


  static bool P1LeftofP2(const Vec2d& p1, const Vec2d& p2)
  {
    if (p1(X) != p2(X))
      return p1(X) < p2(X);
    else
      return p1(Y) < p2(Y);
  }

  /**
  \brief Performs a counter clockwise radial sort of the input points
         starting with the bottom left point
  @param[out] p Unsorted points or footholds.
  Uses the slow n^2 sort algorithm, shouldn't matter for sorting 4 points :)
  Fails when 3 points are on same line and one could be removed
   */
  static std::vector<size_t>
  CounterClockwiseSort(const StdVectorEig2d& p)
  {
    // initialize original index locations
    std::vector<size_t> idx(p.size());
    std::iota( idx.begin(), idx.end(), 0 ); // fills vector with sequentially increasing values starting at 0




    // Sort points lexicographically
    // make sure the first point is the one with lowest (x,y) value
    std::sort(idx.begin(), idx.end(),
              [&p](size_t i1, size_t i2) {return P1LeftofP2(p[i1], p[i2]);});



    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    int n = p.size(), k = 0;
    std::vector<size_t> H(2*n);


    // Build lower hull
    for (int i = 0; i < n; ++i) {
      while (k >= 2 && BisRightOfOA(p[H[k-2]], p[H[k-1]], p[idx[i]])) k--;
      H[k++] = idx[i];
    }

    // Build upper hull
    for (int i = n-2, t = k+1; i >= 0; i--) {
      while (k >= t && BisRightOfOA(p[H[k-2]], p[H[k-1]], p[idx[i]])) k--;
      H[k++] = idx[i];
    }

    H.resize(k-1); // remove double start and end point

    return H;




//    // sort counterclockwise
//    Vec2d line_start = p[idx[0]];
//    for (size_t i=1; i<idx.size()-1; i++)
//      for (size_t j=i+1; j<idx.size(); j++) {
//        Vec2d line_end   = p[idx[i]];
//        Vec2d point      = p[idx[j]];
//        if (BisRightOfOA(line_start, line_end, point))
//          std::swap(idx[i], idx[j]);
//      }
//    return idx;




  }

};


} // namespace utils
} // namespace xpp

#endif // _XPP_UTILS_POINT2D_MANIPULATOR_H_
