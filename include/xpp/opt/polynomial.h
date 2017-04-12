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

#include <array>     // std::array
#include <stdexcept>

#include <xpp/state.h>

namespace xpp {
namespace opt {

/** Constructs a polynomial given start and end states.
  *
  * The polynomial types are:
  * Linear:  Et + F;
  * Cubic:   Ct^3 + Dt^2 + Et + F;
  * Quintic: At^5 + Bt^4 + Ct^3 + Dt^2 + Et + F
  */
class Polynomial {
public:

  // f = At^5 + Bt^4 + Ct^3 + Dt^2 + Et + f
  enum PolynomialCoeff { A=0, B, C, D, E, F };
  static constexpr std::array<PolynomialCoeff, 6> AllSplineCoeff = {{A,B,C,D,E,F}};

  enum PointType {Start=0, Goal=1};

public:
  Polynomial();
  virtual ~Polynomial() {};

  /**
   * @brief Sets the starting point of the spline and the end position.
   * @param T Time to go from start to end.
   * @param start_p desired start position, velocity and acceleration.
   * @param end_p desired goal position, velocity and acceleration.
   */
  void SetBoundary(double T, const StateLin1d& start, const StateLin1d& end);

  /**
   * @brief Sets the starting point of the spline and the end position.
   * @param dt current spline time.
   * @param point current position at time dt.
   */
  bool GetPoint(const double dt, StateLin1d& point) const;

  double GetCoefficient(PolynomialCoeff coeff) const;
  void SetCoefficient(PolynomialCoeff coeff, double value);

  double GetDuration() const;

protected:
  double duration;
  std::array< double, AllSplineCoeff.size() > c; //!< coefficients of spline

private:
  /**
   * @brief Calculates all spline coeff of current spline.
   *
   * params are the same as @ref getPoint.
   * This is the only function that must be implemented by the child classes.
   */
  virtual void SetPolynomialCoefficients(double T, const StateLin1d& start_p, const StateLin1d& end_p) = 0;
};

inline Polynomial::Polynomial()
{
  for (double& _c : c) _c = 0.0; /** set to zero so low-order Polynomials don't use. */
  duration = 0.0;
}

inline void Polynomial::SetBoundary(double T, const StateLin1d& start_p, const StateLin1d& end_p)
{
  if(T <= 0.0)
    throw std::invalid_argument("Cannot create a Polynomial with zero or negative duration");

  duration = T;
  SetPolynomialCoefficients(T, start_p, end_p);
}

/**
 * @ingroup Polynomials
 * \anchor polynomials ready to use.
 * @{
 */
class LinearPolynomial : public Polynomial {
public:
  LinearPolynomial() {};
  ~LinearPolynomial() {};

  static int GetNumCoeff() { return 2; }; //E,F

private:
  void SetPolynomialCoefficients(double T, const StateLin1d& start, const StateLin1d& end);
};

/** @brief a polynomial of the form ct^3 + dt^2 + et + f.
 * see matlab script "third_order_poly.m" for generation of these values.
 */
class CubicPolynomial : public Polynomial {
public:
  CubicPolynomial() {};
  ~CubicPolynomial() {};

  static int GetNumCoeff() { return 4; }; //C,D,E,F

  // spring_clean_ move up to base class?
  double GetDerivativeOfPosWrtPos(double t, PointType p) const;

private:
  void SetPolynomialCoefficients(double T, const StateLin1d& start, const StateLin1d& end);
};

class QuinticPolynomial : public Polynomial {
public:
  QuinticPolynomial() {};
  ~QuinticPolynomial() {};

  static int GetNumCoeff() { return 6; }; //A,B,C,D,E,F

private:
  void SetPolynomialCoefficients(double T, const StateLin1d& start, const StateLin1d& end);
};

/** @brief Creates a smooth up and down motion for e.g. swinging a leg.
 *
 * see matlab script "swingleg_z_height.m" for generation of these values.
 */
class LiftHeightPolynomial : public Polynomial {
public:
  LiftHeightPolynomial() {};
  ~LiftHeightPolynomial() {};

  static int GetNumCoeff() { return 6; }; //A,B,C,D,E,F

  /** Determines how quick the height rises/drops.
   *
   * h is not the exact height between the contact points, but the height
   * that the swingleg has as 1/n_*T and (n-1)/n*T, e.g. shortly after lift-off
   * and right before touchdown. The lift-height in the center is higher.
   */
  void SetShape(int n, double h);

private:
  int n_ = 6;        ///< determines the shape of the swing motion
  double h_ = 0.03;  ///< proportional to the lift height between contacts
  void SetPolynomialCoefficients(double T, const StateLin1d& start, const StateLin1d& end);
};
/** @} */

} // namespace opt
} // namespace xpp

#endif // _XPP_OPT_UTILS_POLYNOMIAL_H_
