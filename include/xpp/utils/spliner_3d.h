/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#ifndef _XPP_UTILS_SPLINER_3D_H_
#define _XPP_UTILS_SPLINER_3D_H_

#include "spliner.h"
#include <xpp/utils/cartesian_declarations.h>
#include <xpp/utils/base_state.h>
#include <Eigen/Dense>

namespace xpp {
namespace utils {

// cmo move this inside Quintic Spliner class
static const int kCoeffCount = 6;
enum SplineCoeff { A=0, B, C, D, E, F };

// cmo rename this to "2dCoeff" of smth
struct CoeffValues {
  double x[kCoeffCount];
  double y[kCoeffCount];
  CoeffValues()
  {
    for (int c = A; c <= F; ++c)
      x[c] = y[c] = 0.0;
  };

  CoeffValues(double xa, double xb, double xc, double xd, double xe, double xf,
              double ya, double yb, double yc, double yd, double ye, double yf)
  {
    x[A] = xa; x[B] = xb; x[C] = xc; x[D] = xd; x[E] = xe; x[F] = xf;
    y[A] = ya; y[B] = yb; y[C] = yc; y[D] = yd; y[E] = ye; y[F] = yf;
  }

  /** generates random spline coefficients between -25 and 25 */
  void SetRandom()
  {
    for (int c = A; c <= F; ++c) {
      x[c] = (double)rand() / RAND_MAX * 50 - 25;
      y[c] = (double)rand() / RAND_MAX * 50 - 25;
    }
  }
};



template<typename SplineType>
class Spliner2d {
public:
  using Point = utils::BaseLin2d;
  using Vector2d = Eigen::Vector2d;

public:
  explicit Spliner2d() {};
  virtual ~Spliner2d() {};
  void SetBoundary(double T, const Point& start, const Point& end);
  bool GetPoint(const double dt, Point& p) const;

  Vector2d GetState(MotionDerivative pos_vel_acc_jerk, double t) const;
  double GetCoefficient(int dim, SplineCoeff coeff) const;
  void SetSplineCoefficients(const CoeffValues &coeff_values = CoeffValues());

private:
  SplineType splineX, splineY;
};


/**
@brief Creates 3 dimensional (x,y,z) spline from start to end with duration T.

The type of spline (linear, cubic, quintic) is set as a template parameter.
 */
template<typename SplineType>
class Spliner3d {

  SplineType splineX, splineY, splineZ;

public:
typedef utils::BaseLin3d Point;

public:
  explicit Spliner3d() {};
  virtual ~Spliner3d() {};
  void SetBoundary(double T, const Point& start, const Point& end);
  bool GetPoint(const double dt, Point& p) const;
};


} // namespace utils
} // namespace xpp

#endif // _XPP_UTILS_SPLINER_3D_H_
