/**
@file    point2d_manipulations.h
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Utilites for creating lines with points, calculating distances,...
 */

#ifndef _XPP_UTILS_POINT2D_MANIPULATOR_H_
#define _XPP_UTILS_POINT2D_MANIPULATOR_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp/utils/eigen_std_conversions.h>
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
public:
  using StdVectorEig2d = xpp::utils::StdVecEigen2d;
  using Vec2d = Eigen::Vector2d;

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
   * Sorts the points in p from left to right (tie: bottom to top) obtaining the indexes.
   * @param p 2d points to be sorted
   * @return the sorted indexes so the value, e.g. i=idx[0] corresponds to point p[i]
   * @link http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
   */
  static std::vector<size_t> SortIndexesLeftToRight(const StdVectorEig2d& p)
      {
    // index vector where the number corresponds to the point at that index  in p
    std::vector<size_t> idx(p.size());
    std::iota( idx.begin(), idx.end(), 0 ); // fills vector with sequentially increasing values starting at 0

    // Sort points lexicographically
    // make sure the first point is the one with lowest (x,y) value
    std::sort(idx.begin(), idx.end(),
              [&p](size_t i1, size_t i2) {return P1LeftofP2(p[i1], p[i2]);});

    return idx;
      }

  /**
   * Extracts the points in p that define the convex hull and sorts them counterclockwise
   * @param p 2d points to be sorted
   * @return the sorted indexes so the value, e.g. i=idx[0] corresponds to point p[i]
   * @link https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
   */
  static std::vector<size_t>
  BuildConvexHullCounterClockwise(const StdVectorEig2d& p)
  {
    std::vector<size_t> idx;

    // If all points are equal convex hull is just one of the points
    if (std::equal(p.begin()+1, p.end(), p.begin())) {
      idx.push_back(0);
      return idx;
    }

    idx = SortIndexesLeftToRight(p);

    // Returns a list of points on the convex hull in counter-clockwise order.
    int n = p.size(), k = 0;
    std::vector<size_t> idx_hull(2*n);

    // Build lower hull
    for (int i = 0; i < n; ++i) {
      while (k >= 2 && BisRightOfOA(p[idx_hull[k-2]], p[idx_hull[k-1]], p[idx[i]])) k--;
      idx_hull[k++] = idx[i];
    }

    // Build upper hull
    for (int i = n-2, t = k+1; i >= 0; i--) {
      while (k >= t && BisRightOfOA(p[idx_hull[k-2]], p[idx_hull[k-1]], p[idx[i]])) k--;
      idx_hull[k++] = idx[i];
    }

    idx_hull.resize(k-1); // remove double start and end point
//    if (p[idx_hull[0]] == p[idx_hull[1]])
//      idx_hull.resize(1);

    return idx_hull;

  }

};


} // namespace utils
} // namespace xpp

#endif // _XPP_UTILS_POINT2D_MANIPULATOR_H_
