/**
@file    polynomial_xd.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates and x-dimensional spline from start to end with duration T
 */

#ifndef _XPP_UTILS_POLYNOMIALXD_H_
#define _XPP_UTILS_POLYNOMIALXD_H_

#include <xpp/utils/cartesian_declarations.h>
#include <xpp/utils/polynomial.h>
#include <Eigen/Dense>

namespace xpp {
namespace utils {


template<typename PolynomialType, size_t N_DIM, typename PointType>
class PolynomialXd {
public:
  using Point = PointType;
  using Vector = Eigen::Matrix<double,N_DIM,1>;
  static const int kNumDim = N_DIM;

public:
  explicit PolynomialXd() : id_(0) {};
  virtual ~PolynomialXd() {};
  void SetBoundary(double T, const Point& start, const Point& end);
  bool GetPoint(const double dt, Point& p) const;

  Vector GetState(MotionDerivative pos_vel_acc_jerk, double t) const;
  double GetCoefficient(int dim, SplineCoeff coeff) const;
  void   SetCoefficients(int dim, SplineCoeff coeff, double value);

  void SetDuration(double duration);
  double GetDuration() const;

  uint GetId()            const { return id_; };
  void SetId(uint id)           { id_ = id;   };

private:
  std::array<PolynomialType, N_DIM> polynomials_; ///< X,Y,Z dimensions
  uint id_; // to identify the order relative to other polynomials
};

} // namespace utils
} // namespace xpp

#include "impl/polynomial_xd-impl.h"

#endif // _XPP_UTILS_POLYNOMIALXD_H_
