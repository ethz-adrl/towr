/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef TOWR_VARIABLES_SPLINE_H_
#define TOWR_VARIABLES_SPLINE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/state.h>

#include <ifopt/composite.h> // for Jacobian definition


namespace towr {


/** @brief Wraps a sequence of polynomials with optimized coefficients.
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a spline with position/velocity and acceleration.
  */
class Spline {
public:
  using Ptr        = std::shared_ptr<Spline>;
  using VecTimes   = std::vector<double>;
  using LocalInfo  = std::pair<int,double>; ///< id and local time
  using StateLinXd = xpp::StateLinXd;
  using MotionDerivative = xpp::MotionDerivative;

  Spline () = default;
  virtual ~Spline () = default;

  static int GetSegmentID(double t_global, const VecTimes& durations);
  static LocalInfo GetLocalTime(double t_global, const VecTimes& durations);

  virtual const StateLinXd GetPoint(double t_global) const = 0;

  virtual opt::Component::Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const = 0;
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_SPLINE_H_ */
