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

#include <stdexcept>
#include <iostream>
#include <array>  // std::array
#include <limits> // std::numeric_limits
#include <cmath>  // std::nextafter()

namespace xpp {
namespace utils {

/**
 * @class Polynomial
 * @addtogroup Polynomial
 *
 * @brief Splines points depending on the boundary conditions set.
 *
 * @b Example @b usage:
 *
 * 1. Create Object:
 * \code
 * iit::commons::LinearPolynomial ls;
 * \endcode
 *
 * 2. Set boundary conditions for a specific polynomial:
 * \code
 * double max_time = 4.0;
 * Point start(3.0, 0.0, 0.0); // pos, vel, acc
 * Point goal (4.0, 1.0, 0.0); // pos, vel, acc
 * ls.SetBoundary(max_time, start, goal)
 * \endcode
 *
 * 3. Get point of polynomial at specific time:
 * \code
 * iit::commons::Polynomial::Point curr;
 * double t_curr = 3.5;
 * ls.GetPoint(t_curr, curr);
 * \endcode
 *
 * For list of available Polynomials see \ref Polynomials "available Polynomials".
 */
class Polynomial {
public:

  // f = At^5 + Bt^4 + Ct^3 + Dt^2 + Et + f
  enum PolynomialCoeff { A=0, B, C, D, E, F };
  static constexpr std::array<PolynomialCoeff, 6> AllSplineCoeff = {{A,B,C,D,E,F}};

  struct Point1d {
    double x;    ///< position
    double xd;   ///< velocity
    double xdd;  ///< acceleration
    double xddd; ///< jerk
    Point1d() : x(0.0), xd(0.0), xdd(0.0), xddd(0.0) {}
  };

public:
  Polynomial();
  virtual ~Polynomial() {};

  /**
   * @brief Sets the starting point of the spline and the end position.
   * @param T Time to go from start to end.
   * @param start_p desired start position, velocity and acceleration.
   * @param end_p desired goal position, velocity and acceleration.
   */
  void SetBoundary(double T, const Point1d& start, const Point1d& end);

  /**
   * @brief Sets the starting point of the spline and the end position.
   * @param dt current spline time.
   * @param point current position at time dt.
   */
  bool GetPoint(const double dt, Point1d& point) const;

  std::array< double, AllSplineCoeff.size() > c; //!< coefficients of spline
  double duration;
private:
  /**
   * @brief Calculates all spline coeff of current spline.
   *
   * params are the same as @ref getPoint.
   * This is the only function that must be implemented by the child classes.
   */
  virtual void SetPolynomialCoefficients(double T, const Point1d& start_p, const Point1d& end_p) = 0;

protected:
};

inline Polynomial::Polynomial()
{
  for (double& _c : c) _c = 0.0; /** set to zero so low-order Polynomials don't use. */
  duration = 0.0;
}

inline void Polynomial::SetBoundary(double T, const Point1d& start_p, const Point1d& end_p)
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
  void SetPolynomialCoefficients(double T, const Point1d& start, const Point1d& end);
};

class CubicPolynomial : public Polynomial {
public:
  CubicPolynomial() {};
  ~CubicPolynomial() {};

  static int GetNumCoeff() { return 4; }; //C,D,E,F

private:
  void SetPolynomialCoefficients(double T, const Point1d& start, const Point1d& end);
};

class QuinticPolynomial : public Polynomial {
public:
  QuinticPolynomial() {};
  ~QuinticPolynomial() {};

  static int GetNumCoeff() { return 6; }; //A,B,C,D,E,F

private:
  void SetPolynomialCoefficients(double T, const Point1d& start, const Point1d& end);
};
/** @} */

} // namespace utils
} // namespace xpp

#endif // XPP_UTILS_Polynomial_H_
