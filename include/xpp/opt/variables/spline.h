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
  using PPtr    = std::shared_ptr<Polynomial>;
  using VarsPtr = std::shared_ptr<PolynomialVars>;
  using VecP    = std::vector<PPtr>;
  using VecVars = std::vector<VarsPtr>;

  using OptVarsPtr           = Primitive::OptVarsPtr;
  using VecTimes             = std::vector<double>;

  Spline ();
  virtual ~Spline ();

  static int GetSegmentID(double t_global, const std::vector<double>& durations);
  static double GetLocalTime(double t_global, const std::vector<double>& durations);

  static Spline BuildSpline(const OptVarsPtr& opt_vars,
                            const std::string& spline_base_id,
                            const VecTimes& poly_durations);



  // zmp_ nice, these both already correspond to node values :-)
  // but specific for polynomial spline :-(
  const StateLinXd GetPoint(double t_globals) const;
  PPtr GetActivePolynomial(double t_global) const;
  VarsPtr GetActiveVariableSet(double t_global) const;




  // these are the functions that differ
  /** @returns true if the optimization variables poly_vars affect that
   * state of the spline at t_global.
   */
  bool DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const;
  Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const;




  // these are only needed by spline junction constraints
  double GetDurationOfPoly(int id) const { return durations_.at(id); };
  VecP GetPolynomials() const     { return polynomials_; }
  PPtr GetPolynomial(int id) const { return polynomials_.at(id); }


  VecVars GetVarSets() const { return poly_vars_; }



protected:
  VecTimes durations_; ///< duration of each polynomial in spline
  VecP polynomials_;

  // add vector of components as well and separate
  VecVars poly_vars_;

//private:
//  void AddPolynomial(const PolynomialPtr& poly);
//  void SetDurations(const VecTimes& durations);
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
