/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_

#include <memory>
#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/polynomial.h>

namespace xpp {
namespace opt {


/** @brief Wraps a sequence of polynomials with optimized coefficients.
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a spline with position/velocity and acceleration.
  */
class Spline {
public:
  using PPtr      = std::shared_ptr<Polynomial>;
  using VecP      = std::vector<PPtr>;
  using Ptr       = std::shared_ptr<Spline>;

  using OptVarsPtr = Primitive::OptVarsPtr;
  using VecTimes   = std::vector<double>;

  Spline ();
  virtual ~Spline ();

  // zmp_ this is ugly as i only use durations for coeff_spline spline...
  // zmp_ remove duration from here
  static Spline::Ptr BuildSpline(const OptVarsPtr& opt_vars,
                                 const std::string& spline_base_id,
                                 const VecTimes& poly_durations);



  static int GetSegmentID(double t_global, const VecTimes& durations);
  static double GetLocalTime(double t_global, const VecTimes& durations);


  virtual const StateLinXd GetPoint(double t_global) const = 0;

  /** @returns true if the optimization variables poly_vars affect that
   * state of the spline at t_global.
   */
  virtual bool DoVarAffectCurrentState(const std::string& poly_vars,
                                       double t_current) const = 0;
  virtual Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const = 0;
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
