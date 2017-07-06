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

/** @brief Represents a sequence of polynomials with optimized coefficients.
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a spline with position/velocity and acceleration.
  */
class PolynomialSpline : public Component {
public:
  using VecPolynomials = std::vector<Polynomial>;

  PolynomialSpline (const std::string& component_name);
  virtual ~PolynomialSpline ();

  void Init (int n_polys, int poly_order, const VectorXd& initial_value);
  void SetPhaseDurations(const std::vector<double>& phase_durations,
                         int polys_per_duration);

  VectorXd GetValues () const override;
  void SetValues (const VectorXd& optimized_coeff) override;

  int Index(int polynomial, int dim, Polynomial::PolynomialCoeff coeff) const;

  /** @brief Calculates the Jacobian at a specific state.
    *
    * @param dxdt whether position, velocity, acceleration or jerk Jacobian is desired
    * @param t_poly the time at which the Jacobian is desired, expressed since current polynomial is active.
    * @param id the ID of the current polynomial
    * @param dim in which dimension (x,y) the Jacobian is desired.
    */
  JacobianRow GetJacobianWrtCoeffAtPolynomial(MotionDerivative dxdt,
                                              double t_poly, int id,
                                              int dim) const;
  JacobianRow GetJacobianWrtCoeffAtPolynomial(MotionDerivative dxdt,
                                              int id, double t_poly,
                                              int dim) const = delete;

  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, int dim) const;
  Jacobian    GetJacobian(double t_global, MotionDerivative dxdt) const;
  Jacobian    GetJacobian(int id, double t_local, MotionDerivative dxdt) const;

  VecPolynomials GetPolynomials() const { return polynomials_; }
  int GetNDim() const {return n_dim_; };


  const StateLinXd GetPoint(double t_globals) const;
  const StateLinXd GetPoint(int id, double t_local) const;
  int GetSegmentID(double t_global) const;
  double GetLocalTime(double t_global) const;
  double GetTotalTime() const;


  double GetDurationOfPoly(int id) const { return durations_.at(id); };

protected:
  std::vector<double> durations_; ///< duration of each polynomial in spline
  VecPolynomials polynomials_;    ///< pointer to retain access to polynomial functions
  int n_dim_;

  int n_polys_per_phase_; // polynomials used to represent each timing phase

  int GetFreeCoeffPerPoly() const;
};



class EndeffectorSpline : public PolynomialSpline {
public:
  EndeffectorSpline(const std::string& id, bool first_phase_in_contact);
  virtual ~EndeffectorSpline ();

  virtual VecBound GetBounds () const override;

  // shouldn't be allowed to use this function for now
//  Jacobian GetJacobian(double t_global, MotionDerivative dxdt) = delete;

private:
  bool first_phase_in_contact_;
};

// zmp_ DRY almost the same as above, combine
class ForceSpline : public PolynomialSpline {
public:
  ForceSpline(const std::string& id, bool first_phase_in_contact, double max_force);
  virtual ~ForceSpline ();

  virtual VecBound GetBounds () const override;

private:
  bool first_phase_in_contact_;
  double max_force_;
};







} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
