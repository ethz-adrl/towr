/**
 @file    coeff_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_COEFF_SPLINE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_COEFF_SPLINE_H_

#include <memory>
#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/composite.h>
#include <xpp/polynomial.h>
#include <xpp/state.h>

#include "spline.h"

namespace xpp {
namespace opt {

// doesn't hold any variables, just for keeping the struct in optimization
// variables
class CoeffSpline : public Spline, public Component {
public:
  using VarsPtr   = std::shared_ptr<PolynomialVars>;
  using VecVars   = std::vector<VarsPtr>;


  CoeffSpline(const std::string& spline_base_id,
              const VecTimes& poly_durations);
  virtual ~CoeffSpline();


  // linear interpolation from start to goal
  void InitializeVariables(const VectorXd& initial_pos, const VectorXd& final_pos);


  virtual const StateLinXd GetPoint(double t_global) const override;
  virtual bool DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const override;
  virtual Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const override;


  int GetPolyCount() const { return poly_vars_.size(); };
  double GetDurationOfPoly(int id) const { return durations_.at(id); };


  // these are critical to expose... try not to
  Polynomial::Ptr GetPolynomial(int id) const { return poly_vars_.at(id)->GetPolynomial(); }
  VarsPtr GetVarSet(int id) const { return poly_vars_.at(id); }

  VecVars poly_vars_;  ///< the opt. variables that influence the polynomials
private:
  VarsPtr GetActiveVariableSet(double t_global) const;

  VecTimes durations_; ///< duration of each polynomial in spline
};

} /* namespace opt */
} /* namespace xpp */


#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
