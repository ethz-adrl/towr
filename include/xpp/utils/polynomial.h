/**
@file   spliner.h
@author Alexander Winkler (winklera@ethz.ch)
@date   29.07.2014

@brief  A virtual class spliner with ready to use derived spliners

Spliners ready to use:
        - Linear Spliner
        - Cubic Spliner
        - Quintic Spliner
*/
#ifndef _XPP_UTILS_SPLINER_H_
#define _XPP_UTILS_SPLINER_H_

#include <stdexcept>
#include <iostream>
#include <array>  // std::array
#include <limits> // std::numeric_limits
#include <cmath>  // std::nextafter()

namespace xpp {
namespace utils {

static const int kCoeffCount = 6;
enum SplineCoeff { A=0, B, C, D, E, F };
static const std::array<SplineCoeff, kCoeffCount> AllSplineCoeff = {A,B,C,D,E,F};

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
 * iit::commons::LinearSpliner ls;
 * \endcode
 *
 * 2. Set boundary conditions for a specific spline:
 * \code
 * double max_time = 4.0;
 * iit::commons::Spliner::Point start(3.0, 0.0, 0.0); // pos, vel, acc
 * iit::commons::Spliner::Point goal (4.0, 1.0, 0.0); // pos, vel, acc
 * ls.SetBoundary(max_time, start, goal)
 * \endcode
 *
 * 3. Get point of spline at specific time:
 * \code
 * iit::commons::Spliner::Point curr;
 * double t_curr = 3.5;
 * ls.GetPoint(t_curr, curr);
 * \endcode
 *
 * For list of available spliners see \ref spliners "available spliners".
 */
class Polynomial {
public:

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

  std::array< double, kCoeffCount > c; //!< coefficients of spline
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
  for (double& _c : c) _c = 0.0; /** set to zero so low-order spliners don't use. */
  duration = 0.0;
}

inline void Polynomial::SetBoundary(double T, const Point1d& start_p, const Point1d& end_p)
{
  if(T <= 0.0)
    throw std::invalid_argument("Cannot create a spliner with zero or negative duration");

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
private:
  void SetPolynomialCoefficients(double T, const Point1d& start, const Point1d& end);
};

class CubicPolynomial : public Polynomial {
public:
  CubicPolynomial() {};
  ~CubicPolynomial() {};
private:
  void SetPolynomialCoefficients(double T, const Point1d& start, const Point1d& end);
};

class QuinticPolynomial : public Polynomial {
public:
  QuinticPolynomial() {};
  ~QuinticPolynomial() {};
private:
  void SetPolynomialCoefficients(double T, const Point1d& start, const Point1d& end);
};
/** @} */

} // namespace utils
} // namespace xpp

#endif // XPP_UTILS_SPLINER_H_
