/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_

#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include "polynomial_xd.h"
#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

/** Represents the Center of Mass (CoM) motion as a Spline (sequence of polynomials).
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a CoM position/velocity and acceleration.
  */
class ComSpline : public BaseMotion {
public:
  using Derivatives    = std::vector<MotionDerivative>;
  using PolyCoeff      = Polynomial::PolynomialCoeff;
  using PolyXdT        = PolynomialXd<QuinticPolynomial, StateLin2d>;
  using PolyHelpers    = PolyVecManipulation<PolyXdT>;
  using VecPolynomials = PolyHelpers::VecPolynomials;

  ComSpline ();
  virtual ~ComSpline ();

  void Init(double t_global, int polynomials_per_second);

  StateLin2d GetCom(double t_global) const override;
  double GetTotalTime() const override;
  VectorXd GetXYSplineCoeffients () const override;
  int GetTotalFreeCoeff() const;

  int Index(int polynomial, Coords3D dim, PolyCoeff coeff) const;


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
                                              Coords3D dim) const;
  JacobianRow GetJacobianWrtCoeffAtPolynomial(MotionDerivative dxdt,
                                              int id, double t_poly,
                                              Coords3D dim) const = delete;

  VecPolynomials GetPolynomials() const { return polynomials_; }
  void SetSplineXYCoefficients (const VectorXd& optimized_coeff) override;
  void SetCoefficientsZero();

private:
  VecPolynomials polynomials_;
  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, Coords3D dim) const override;

  // careful: assumes all splines (X,Y,1,..,n) same type
  int NumFreeCoeffPerSpline() const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_ */
