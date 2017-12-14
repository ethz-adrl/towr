/**
 @file    coeff_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_COEFF_SPLINE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_COEFF_SPLINE_H_

#include <string>
#include <vector>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/state.h>

#include <ifopt/composite.h>
#include <xpp_opt/polynomial.h>
#include <xpp_opt/variables/spline.h>


namespace xpp {

// doesn't hold any variables, just for keeping the struct in optimization
// variables
class CoeffSpline : public Spline, public opt::Variable {
public:
  using VecVars  = std::vector<PolynomialVars::Ptr>;
  using VectorXd = Eigen::VectorXd;

  CoeffSpline(const std::string& spline_base_id,
              const VecTimes& poly_durations);
  virtual ~CoeffSpline();


  // linear interpolation from start to goal
  void InitializeVariables(const VectorXd& initial_pos, const VectorXd& final_pos);


  virtual const StateLinXd GetPoint(double t_global) const override;
  virtual bool HoldsVarsetThatIsActiveNow(const std::string& poly_vars, double t_global) const override;
  virtual Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const override;


  int GetPolyCount() const { return poly_vars_.size(); };
  double GetDurationOfPoly(int id) const { return durations_.at(id); };


  // these are critical to expose... try not to
  Polynomial::Ptr GetPolynomial(int id) const { return poly_vars_.at(id)->GetPolynomial(); }
  PolynomialVars::Ptr GetVarSet(int id) const { return poly_vars_.at(id); }


  virtual VecBound GetBounds() const override { return VecBound(GetRows(), opt::NoBound); };
  virtual VectorXd GetValues() const override { assert(false); };
  virtual void SetVariables(const VectorXd& x) override { assert(false); };

  VecVars poly_vars_;  ///< the opt. variables that influence the polynomials
private:
  PolynomialVars::Ptr GetActiveVariableSet(double t_global) const;

  VecTimes durations_; ///< duration of each polynomial in spline
};

} /* namespace xpp */


#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
