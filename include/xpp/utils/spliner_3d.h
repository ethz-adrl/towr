/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#ifndef _XPP_UTILS_SPLINER_3D_H_
#define _XPP_UTILS_SPLINER_3D_H_

#include "spliner.h"
#include <xpp/utils/geometric_structs.h>
#include <Eigen/Dense>

namespace xpp {
namespace utils {

/**
@brief Creates 3 dimensional (x,y,z) spline from start to end with duration T.

The type of spline (linear, cubic, quintic) is set as a template parameter.
 */
template<typename SplineType>
class Spliner3d {

  SplineType splineX, splineY, splineZ;

public:
typedef utils::Point3d Point;

public:
  explicit Spliner3d() {};
  virtual ~Spliner3d() {};
  void SetBoundary(double T, const Point& start, const Point& end);
  bool GetPoint(const double dt, Point& p) const;
};


} // namespace utils
} // namespace xpp

#endif // _XPP_UTILS_SPLINER_3D_H_
