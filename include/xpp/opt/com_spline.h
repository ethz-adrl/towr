/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_

#include <memory>

#include "base_motion.h"
#include "com_polynomial_helpers.h"

namespace xpp {
namespace opt {

/** Represents the Center of Mass (CoM) motion as a Spline (sequence of polynomials).
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a CoM position/velocity and acceleration.
  */
class ComSpline : public BaseMotion {
public:
  using VecPolynomials   = std::vector<ComPolynomial>;
  using Derivatives      = std::vector<MotionDerivative>;
  using PtrS             = std::shared_ptr<ComSpline>;
  using PtrU             = std::unique_ptr<ComSpline>;
  using PolyCoeff        = Polynomial::PolynomialCoeff;

  ComSpline ();
  virtual ~ComSpline ();

  void Init(double t_global, int polynomials_per_second);

  // implements these functions from parent class, now specific for splines
  StateLin2d GetCom(double t_global) const override { return ComPolynomialHelpers::GetCOM(t_global, polynomials_); }
  double GetTotalTime() const override { return ComPolynomialHelpers::GetTotalTime(polynomials_); }
  VectorXd GetXYSplineCoeffients () const override;
  int GetTotalFreeCoeff() const;

  int Index(int polynomial, Coords3D dim, PolyCoeff coeff) const;


  /** The motions (pos,vel,acc) that are fixed by spline structure and cannot
    * be modified through the coefficient values. These will be constrained
    * in the nonlinear program.
    */
  virtual Derivatives GetInitialFreeMotions()  const = 0;
  virtual Derivatives GetJunctionFreeMotions() const = 0;

  int GetPolynomialID(double t_global)  const { return ComPolynomialHelpers::GetPolynomialID(t_global, polynomials_); }
  double GetLocalTime(double t_global)  const { return ComPolynomialHelpers::GetLocalTime(t_global, polynomials_); };
  VecPolynomials GetPolynomials()       const { return polynomials_; }
  ComPolynomial GetPolynomial(size_t i) const { return polynomials_.at(i); }
  ComPolynomial GetLastPolynomial()     const { return polynomials_.back(); };

  /** Calculates the Jacobian at a specific time of the motion, but specified by
    * a local time and a polynome id. This allows to create spline junction constraints
    *
    * @param dxdt whether position, velocity, acceleration or jerk Jacobian is desired
    * @param t_poly the time at which the Jacobian is desired, expressed since current polynomial is active.
    * @param id the ID of the current polynomial
    * @param dim in which dimension (x,y) the Jacobian is desired.
    */
  JacobianRow GetJacobianWrtCoeffAtPolynomial(MotionDerivative dxdt, double t_poly, int id, Coords3D dim) const;

  StateLin2d GetCOGxyAtPolynomial(int id, double t_local) {return ComPolynomialHelpers::GetCOGxyAtPolynomial(id, t_local, polynomials_); };


  void SetCoefficientsZero();

protected:
  VecPolynomials polynomials_;
  void CheckIfSplinesInitialized() const;


private:

  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, Coords3D dim) const override;
  virtual void GetJacobianPos (double t_poly, int id, Coords3D dim, JacobianRow&) const = 0;
  virtual void GetJacobianVel (double t_poly, int id, Coords3D dim, JacobianRow&) const = 0;
  virtual void GetJacobianAcc (double t_poly, int id, Coords3D dim, JacobianRow&) const = 0;
  virtual void GetJacobianJerk(double t_poly, int id, Coords3D dim, JacobianRow&) const = 0;

  virtual JacobianRow GetJacobianVelSquared(double t_global, Coords3D dim) const override;
  virtual JacobianRow GetJacobianPosVelSquared(double t_global, Coords3D dim) const override;
  // only implemented for com_spline_6, throw error otherwise
  virtual void GetJacobianVelSquaredImpl (double t_poly, int id, Coords3D dim, JacobianRow&) const { assert(false); };
  virtual void GetJacobianPosVelSquaredImpl (double t_poly, int id, Coords3D dim, JacobianRow&) const { assert(false); };

  virtual int NumFreeCoeffPerSpline() const = 0;
  virtual std::vector<PolyCoeff> GetFreeCoeffPerSpline() const = 0;

  bool splines_initialized_ = false;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_ */
