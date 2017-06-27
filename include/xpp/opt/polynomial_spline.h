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

#include "polynomial.h"
#include "spline.h"
#include <xpp/opt/constraints/composite.h>

namespace xpp {
namespace opt {

// zmp_ update description
/** Represents the Center of Mass (CoM) motion as a Spline (sequence of polynomials).
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a CoM position/velocity and acceleration.
  */
class PolynomialSpline : public Component, public Spline {
public:
  using State          = StateLinXd;
  using PolyCoeff      = Polynomial::PolynomialCoeff;
  using VecPolynomials = std::vector<std::shared_ptr<Polynomial> >;

  PolynomialSpline (const std::string& component_name = "poly_spline");
  virtual ~PolynomialSpline ();

  void Init(double t_global, double duration_per_polynomial,
            const VectorXd& initial_pos);

  VectorXd GetValues () const override;
  void SetValues (const VectorXd& optimized_coeff) override;

  int Index(int polynomial, int dim, PolyCoeff coeff) const;

  /** Calculates the Jacobian at a specific time of the motion, but specified by
    * a local time and a polynome id. This allows to create spline junction constraints
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

  VecPolynomials GetPolynomials() const { return polynomials_; }
  int GetNDim() const {return n_dim_; };

private:
  VecPolynomials polynomials_; ///< pointer to retain access to polynomial functions
  int n_dim_;

  int GetFreeCoeffPerPoly() const;
  int GetTotalFreeCoeff() const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
