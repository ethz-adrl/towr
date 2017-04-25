/**
@file    polynomial_xd.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates and x-dimensional spline from start to end with duration T
 */

#ifndef _XPP_UTILS_POLYNOMIALXD_H_
#define _XPP_UTILS_POLYNOMIALXD_H_

#include <array>
#include <vector>
#include <Eigen/Dense>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include "polynomial.h"

namespace xpp {
namespace opt {

/** @brief Extends a 1-dimensional polynomial to multiple in e.g. (x,y,z).
 */
template<typename PolynomialType, typename PointType>
class PolynomialXd {
public:
  using PointT = PointType;
  using PolyT  = PolynomialType;
  static const int kNumDim = PointT::kNumDim;
  using Vector = Eigen::Matrix<double,kNumDim,1>;
  using PolyCoeff = typename PolynomialType::PolynomialCoeff;

public:
  explicit PolynomialXd();
  explicit PolynomialXd(double duration);
  virtual ~PolynomialXd();
  void SetBoundary(double T, const PointT& start, const PointT& end);
  PointType GetPoint(const double dt) const;

  Vector GetState(MotionDerivative pos_vel_acc_jerk, double t) const;
  const double GetCoefficient(int dim, PolyCoeff coeff) const;
  void SetCoefficients(int dim, PolyCoeff coeff, double value);

  const double GetDuration() const;
  const PolynomialType GetDim(int dim) const;

private:
  std::array<PolynomialType, kNumDim> polynomials_; ///< X,Y,Z dimensions
};


/** @brief For manipulation of multiple sequential polynomials ("spline").
  */
template<typename TPolyXd>
class PolyVecManipulation {
public:
  using PolynomialXdT  = TPolyXd;
  using PointType      = typename TPolyXd::PointT;
  using VecPolynomials = std::vector<PolynomialXdT>;

  static PointType GetPoint(double t_global, const VecPolynomials& splines);
  static PointType GetPoint(int id, double t_local, const VecPolynomials& splines);
  static PointType GetPoint(double id, int t_local, const VecPolynomials& splines) = delete;   // to generate compiler error when user accidentally mixes up the order

  static int GetPolynomialID(double t_global, const VecPolynomials& splines);
  static double GetTotalTime(const VecPolynomials& splines);
  static double GetLocalTime(double t_global, const VecPolynomials& splines);
};


} // namespace opt
} // namespace xpp

#include "polynomial_xd-impl.h"

#endif // _XPP_UTILS_POLYNOMIALXD_H_
