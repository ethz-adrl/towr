/**
 @file    polynomial_fifth_order.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 3, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_POLYNOMIAL_FIFTH_ORDER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_POLYNOMIAL_FIFTH_ORDER_H_

#include <xpp/cartesian_declarations.h>
#include <Eigen/Dense>

namespace xpp {
namespace opt {

static const int kCoeffCount = 6;
enum SplineCoeff { A=0, B, C, D, E, F };

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


/** Builds a 5th order polynomial from the polynomial coefficients.
  */
class PolynomialFifthOrder {
public:
  typedef Eigen::Vector2d Vec2d;
  using PosVelAcc = MotionDerivative;

  PolynomialFifthOrder();
  PolynomialFifthOrder(const CoeffValues &coeff_values);
  virtual ~PolynomialFifthOrder();

  Vec2d GetState(PosVelAcc whichDeriv, double t) const;
  void SetSplineCoefficients(const CoeffValues &coeff_values = CoeffValues());
  double GetCoefficient(int dim, SplineCoeff coeff) const;

protected:
  double spline_coeff_[kDim2d][kCoeffCount];
  friend class SplineContainerTest_EandFCoefficientTest_Test;

};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_POLYNOMIAL_FIFTH_ORDER_H_ */
