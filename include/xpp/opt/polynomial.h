/**
@file   polynomial.h
@author Alexander Winkler (winklera@ethz.ch)
@date   29.07.2014

@brief  A virtual class Polynomial with ready to use derived Polynomials

Polynomials ready to use:
        - Linear Polynomial
        - Cubic Polynomial
        - Quintic Polynomial
*/
#ifndef _XPP_OPT_UTILS_POLYNOMIAL_H_
#define _XPP_OPT_UTILS_POLYNOMIAL_H_

#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include "spline.h"

namespace xpp {
namespace opt {

/** @brief Constructs a polynomial given start and end states.
  */
class Polynomial : public Segment {
public:

  // x(t) =   Ft^5 +   Et^4 +  Dt^3 +  Ct^2 + Bt + A
  // x(t) =  5Ft^4 +  4Et^3 + 3Dt^2 + 2Ct   + B
  // x(t) = 20Ft^3 + 12Et^2 + 6Dt   + 2C
  enum PolynomialCoeff { A=0, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W};
  using CoeffVec = std::vector<PolynomialCoeff>;

  enum PointType {Start=0, Goal=1};

public:
  Polynomial(int order);
  virtual ~Polynomial() {};

  /**
   * @brief Sets the starting point of the spline and the end position.
   * @param T Time to go from start to end.
   * @param start_p desired start position, velocity and acceleration.
   * @param end_p desired goal position, velocity and acceleration.
   */
  void SetBoundary(double T, const StateLinXd& start, const StateLinXd& end);

  /**
   * @brief Sets the starting point of the spline and the end position.
   * @param dt current spline time.
   * @param point current position at time dt.
   */
  StateLinXd GetPoint(const double dt) const;
  double GetDerivativeWrtCoeff(MotionDerivative deriv,
                               PolynomialCoeff coeff,
                               double t) const;
  virtual double GetDerivativeOfPosWrtPos(double t, PointType p) const { assert(false); };

  double GetCoefficient(int dim, PolynomialCoeff coeff) const;
  void SetCoefficient(int dim,   PolynomialCoeff coeff, double value);

  CoeffVec GetCoeffIds() const;
  double GetDuration() const;

  void UpdateCoefficients();

  StateLinXd start_, end_;
protected:
  double duration = 0.0;
  std::vector<VectorXd> coeff_; //!< coefficients of spline
  CoeffVec coeff_ids_;

private:
  /**
   * @brief Calculates all spline coeff of current spline.
   *
   * params are the same as @ref getPoint.
   * This is the only function that must be implemented by the child classes.
   */
  virtual void SetPolynomialCoefficients(double T,
                                         const StateLinXd& start_p,
                                         const StateLinXd& end_p) = 0;
};

/** Zero-order hold x(t) = A;
  */
class ConstantPolynomial: public Polynomial {
public:
  ConstantPolynomial() : Polynomial(0) {};
  ~ConstantPolynomial() {};

  double GetDerivativeOfPosWrtPos(double t, PointType p) const override;

private:
  void SetPolynomialCoefficients(double T, const StateLinXd& start, const StateLinXd& end);
};

class LinearPolynomial : public Polynomial {
public:
  LinearPolynomial() : Polynomial(1) {};
  ~LinearPolynomial() {};

  double GetDerivativeOfPosWrtPos(double t, PointType p) const override;

private:
  void SetPolynomialCoefficients(double T, const StateLinXd& start, const StateLinXd& end);
};

/** @brief a polynomial of the form ct^3 + dt^2 + et + f.
 * see matlab script "third_order_poly.m" for generation of these values.
 */
class CubicPolynomial : public Polynomial {
public:
  CubicPolynomial() : Polynomial(3) {};
  ~CubicPolynomial() {};

  double GetDerivativeOfPosWrtPos(double t, PointType p) const override;

private:
  void SetPolynomialCoefficients(double T, const StateLinXd& start, const StateLinXd& end);
};

class QuarticPolynomial : public Polynomial {
public:
  QuarticPolynomial() : Polynomial(4) {};
  ~QuarticPolynomial() {};

private:
  void SetPolynomialCoefficients(double T, const StateLinXd& start,
                                 const StateLinXd& end);
};

class QuinticPolynomial : public Polynomial {
public:
  QuinticPolynomial() : Polynomial(5) {};
  ~QuinticPolynomial() {};

private:
  void SetPolynomialCoefficients(double T, const StateLinXd& start, const StateLinXd& end);
};


/** @brief Creates a smooth up and down motion for e.g. swinging a leg.
 *
 * see matlab script "swingleg_z_height.m" for generation of these values.
 */
class LiftHeightPolynomial : public Polynomial {
public:
  LiftHeightPolynomial() :Polynomial(5) {};
  ~LiftHeightPolynomial() {};

  /** Determines how quick the height rises/drops.
   *
   * h is not the exact height between the contact points, but the height
   * that the swingleg has as 1/n_*T and (n-1)/n*T, e.g. shortly after lift-off
   * and right before touchdown. The lift-height in the center is higher.
   */
  void SetShape(int n, double h);

private:
  int n_ = 6;        ///< determines the shape of the swing motion
  double height_ = 0.03;  ///< proportional to the lift height between contacts
  void SetPolynomialCoefficients(double T, const StateLinXd& start, const StateLinXd& end);
};

} // namespace opt
} // namespace xpp

#endif // _XPP_OPT_UTILS_POLYNOMIAL_H_
