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



  const StateLinXd GetPoint(double t_globals) const;


  // these are the functions that differ
  /** @returns true if the optimization variables poly_vars affect that
   * state of the spline at t_global.
   */
  virtual bool DoVarAffectCurrentState(const std::string& poly_vars,
                                       double t_current) const = 0;
  virtual Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const = 0;







protected:

//  void AddPolynomial(PPtr p) { polynomials_.push_back(p); };


  int GetSegmentID(double t_global) const;
  double GetLocalTime(double t_global) const;

  VecTimes durations_; ///< duration of each polynomial in spline
  VecP polynomials_;   ///< the polynomials

private:
  PPtr GetActivePolynomial(double t_global) const;
};




class CoeffSpline : public Spline {
public:

  using VarsPtr   = std::shared_ptr<PolynomialVars>;
  using VecVars   = std::vector<VarsPtr>;

  // factory method
  static Spline::Ptr BuildSpline(const OptVarsPtr& opt_vars,
                                 const std::string& spline_base_id,
                                 const VecTimes& poly_durations);



  // these are the functions that differ
  /** @returns true if the optimization variables poly_vars affect that
   * state of the spline at t_global.
   */
  virtual bool DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const override;
  virtual Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const override;
  VarsPtr GetActiveVariableSet(double t_global) const;


  int GetPolyCount() const { return polynomials_.size(); };
  double GetDurationOfPoly(int id) const { return durations_.at(id); };


  // these are critical to expose... try not to
  PPtr GetPolynomial(int id) const { return polynomials_.at(id); }
  VarsPtr GetVarSet(int id) const { return poly_vars_.at(id); }

private:
  VecVars poly_vars_;  ///< the opt. variables that influence the polynomials




};



} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
