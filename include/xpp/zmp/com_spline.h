/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_

#include "com_motion.h"
#include "com_polynomial.h"
#include <memory>

namespace xpp {
namespace zmp {

/** Represents the Center of Mass (CoM) motion as a Spline (sequence of polynomials).
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a CoM position/velocity and acceleration.
  */
class ComSpline : public ComMotion {
public:
  typedef std::vector<ComPolynomial> VecPolynomials;
  typedef xpp::utils::MotionDerivative MotionDerivative;
  typedef xpp::utils::VecScalar VecScalar;
  typedef xpp::utils::BaseLin2d Point2d;
  typedef xpp::utils::Coords3D Coords3D;
  typedef std::vector<MotionDerivative> Derivatives;
  typedef std::shared_ptr<ComSpline> PtrS;
  typedef std::unique_ptr<ComSpline> PtrU;


  ComSpline ();
  virtual ~ComSpline ();

  virtual void Init(const PhaseVec& phases) final;

  // implements these functions from parent class, now specific for splines
  Point2d GetCom(double t_global) const override { return ComPolynomial::GetCOM(t_global, polynomials_); }
  double GetTotalTime() const override { return ComPolynomial::GetTotalTime(polynomials_); }
  int GetTotalFreeCoeff() const override;
  VectorXd GetCoeffients () const override;

  int Index(int polynomial, Coords3D dim, SplineCoeff coeff) const;


  /** The motions (pos,vel,acc) that are fixed by spline structure and cannot
    * be modified through the coefficient values. These will be constrained
    * in the nonlinear program.
    */
  virtual Derivatives GetInitialFreeMotions()  const = 0;
  virtual Derivatives GetJunctionFreeMotions() const = 0;

  int GetPolynomialID(double t_global)  const { return ComPolynomial::GetPolynomialID(t_global, polynomials_); }
  double GetLocalTime(double t_global)  const { return ComPolynomial::GetLocalTime(t_global, polynomials_); };
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

  Point2d GetCOGxyAtPolynomial(int id, double t_local) {return ComPolynomial::GetCOGxyAtPolynomial(id, t_local, polynomials_); };



protected:
  VecPolynomials polynomials_;
  void CheckIfSplinesInitialized() const;


private:

  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, Coords3D dim) const override;
  virtual void GetJacobianPos (double t_poly, int id, Coords3D dim, JacobianRow&) const = 0;
  virtual void GetJacobianVel (double t_poly, int id, Coords3D dim, JacobianRow&) const = 0;
  virtual void GetJacobianAcc (double t_poly, int id, Coords3D dim, JacobianRow&) const = 0;
  virtual void GetJacobianJerk(double t_poly, int id, Coords3D dim, JacobianRow&) const = 0;

  virtual int NumFreeCoeffPerSpline() const = 0;
  virtual std::vector<SplineCoeff> GetFreeCoeffPerSpline() const = 0;

  bool splines_initialized_ = false;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_ */
