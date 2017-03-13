/**
@file   Polynomial.h
@author Alexander Winkler (winklera@ethz.ch)
@date   29.07.2014

@brief  A virtual class Polynomial with ready to use derived Polynomials

Polynomials ready to use:
        - Linear Polynomial
        - Cubic Polynomial
        - Quintic Polynomial
*/
#ifndef _XPP_UTILS_Polynomial_H_
#define _XPP_UTILS_Polynomial_H_

#include <xpp/state.h>

#include <stdexcept>
#include <iostream>
#include <array>  // std::array

namespace xpp {
namespace opt {

/** Constructs a polynomial given start and end states.
  *
  * The polynomial types are:
  * Linear:  Ex + F;
  * Cubic:   Dx^2 + Ex + F;
  * Quintic: At^5 + Bt^4 + Ct^3 + Dt^2 + Et + f
  */
class Polynomial {
public:

  // f = At^5 + Bt^4 + Ct^3 + Dt^2 + Et + f
  enum PolynomialCoeff { A=0, B, C, D, E, F };
  static constexpr std::array<PolynomialCoeff, 6> AllSplineCoeff = {{A,B,C,D,E,F}};

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

  std::array< double, AllSplineCoeff.size() > c; //!< coefficients of spline
  double duration;
private:
  /**
   * @brief Calculates all spline coeff of current spline.
   *
   * params are the same as @ref getPoint.
   * This is the only function that must be implemented by the child classes.
   */
  virtual void SetPolynomialCoefficients(double T, const StateLin1d& start_p, const StateLin1d& end_p) = 0;

protected:
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

class CubicPolynomial : public Polynomial {
public:
  CubicPolynomial() {};
  ~CubicPolynomial() {};

  static int GetNumCoeff() { return 4; }; //C,D,E,F

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
/** @} */

} // namespace opt
} // namespace xpp

#endif // XPP_UTILS_Polynomial_H_
