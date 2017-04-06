/**
@file    polynomial_xd.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates and x-dimensional spline from start to end with duration T
 */

#ifndef _XPP_UTILS_POLYNOMIALXD_H_
#define _XPP_UTILS_POLYNOMIALXD_H_

#include "polynomial.h"
#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>
#include <Eigen/Dense>

namespace xpp {
namespace opt {


template<typename PolynomialType, typename PointType>
class PolynomialXd {
public:
  using Point = PointType;
  static const int kNumDim = Point::kNumDim;
  using Vector = Eigen::Matrix<double,kNumDim,1>;
  using PolyCoeff = typename PolynomialType::PolynomialCoeff;

public:
  explicit PolynomialXd() {};
  explicit PolynomialXd(int id, double duration);
  virtual ~PolynomialXd();
  void SetBoundary(double T, const Point& start, const Point& end);
  bool GetPoint(const double dt, Point& p) const;

  Vector GetState(MotionDerivative pos_vel_acc_jerk, double t) const;
  double GetCoefficient(int dim, PolyCoeff coeff) const;
  void   SetCoefficients(int dim, PolyCoeff coeff, double value);


  static int GetNumCoeff() { return PolynomialType::GetNumCoeff(); };

  double GetDuration() const;

  uint GetId()            const { return id_; };
  void SetId(uint id)           { id_ = id;   };

  PolynomialType GetDim(int dim) const;


private:
  std::array<PolynomialType, kNumDim> polynomials_; ///< X,Y,Z dimensions
  uint id_; // to identify the order relative to other polynomials
};

} // namespace opt
} // namespace xpp

#include "impl/polynomial_xd-impl.h"

#endif // _XPP_UTILS_POLYNOMIALXD_H_
