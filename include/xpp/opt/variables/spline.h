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
  using PolynomialPtr  = std::shared_ptr<Polynomial>;
  using VecPolynomials = std::vector<PolynomialPtr>;
  using VecTimes       = std::vector<double>;
  using PtrS           = std::shared_ptr<Spline>; // pointer to oneself
  using OptVarsPtr     = Primitive::OptVarsPtr;

  Spline ();
  virtual ~Spline ();

  static PtrS BuildSpline(const OptVarsPtr& opt_vars,
                          const std::string& spline_base_id,
                          const VecTimes& poly_durations);

  static int GetSegmentID(double t_global, const VecTimes&);
  static double GetLocalTime(double t_global, const VecTimes&);

  const StateLinXd GetPoint(double t_globals) const;
  Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const;

  /** @returns true if the polynomial with poly_name is active at current time.
   */
  bool PolynomialActive(const std::string& poly_name, double t_global);
  PolynomialPtr GetActivePolynomial(double t_global) const;


  double GetDurationOfPoly(int id) const { return durations_.at(id); };

  VecPolynomials GetPolynomials() const     { return polynomials_; }
  PolynomialPtr GetPolynomial(int id) const { return polynomials_.at(id); }

protected:
  VecTimes durations_; ///< duration of each polynomial in spline
  VecPolynomials polynomials_;  ///< pointer to retain access to polynomial functions

private:
  void AddPolynomial(const PolynomialPtr& poly);
  void SetDurations(const VecTimes& durations);
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
