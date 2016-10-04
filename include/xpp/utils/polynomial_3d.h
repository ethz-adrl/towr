/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#ifndef _XPP_UTILS_SPLINER_3D_H_
#define _XPP_UTILS_SPLINER_3D_H_

#include <xpp/utils/cartesian_declarations.h>
#include <xpp/utils/base_state.h>
#include <Eigen/Dense>
#include "polynomial.h"

namespace xpp {
namespace utils {


template<typename PolynomialType, size_t N_DIM, typename PointType>
class PolynomialXd {
public:
  using Point = PointType;
  using Vector = Eigen::Matrix<double,N_DIM,1>;
  static const int kNumDim = N_DIM;

public:
  explicit PolynomialXd() {};
  virtual ~PolynomialXd() {};
  void SetBoundary(double T, const Point& start, const Point& end);
  bool GetPoint(const double dt, Point& p) const;

  Vector GetState(MotionDerivative pos_vel_acc_jerk, double t) const;
  double GetCoefficient(int dim, SplineCoeff coeff) const;
  void   SetCoefficients(int dim, SplineCoeff coeff, double value);
  void SetDuration(double duration);

private:
  std::array<PolynomialType, N_DIM> polynomials_; ///< X,Y,Z dimensions
};


/**
@brief Creates 3 dimensional (x,y,z) spline from start to end with duration T.

The type of spline (linear, cubic, quintic) is set as a template parameter.
 */
// cmo remove this class
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

#include "impl/spliner_3d-impl.h"

#endif // _XPP_UTILS_SPLINER_3D_H_
