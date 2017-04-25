/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the ComMotion class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/matrix_vector.h>
#include <xpp/opt/constraints/composite.h>

#include <xpp/opt/com_spline.h>

namespace xpp {
namespace opt {

/** @brief Parameterizes the 6D (x,y,z,r,p,y) base of a system.
  *
  * This class is responsible for providing a common interface to represent
  * the motion of a system. Specific parametrizations can for example use
  * splines or solutions of the Equation of Motion as representation.
  */
class BaseMotion : public Component {
public:
  using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;
  using PtrS        = std::shared_ptr<BaseMotion>;
  using PtrU        = std::unique_ptr<BaseMotion>;

  BaseMotion ();
  virtual ~BaseMotion ();

  void AddComSpline(const ComSpline&);

  virtual VectorXd GetValues() const override;
  virtual void SetValues(const VectorXd&) override;


  State3d GetBase(double t_global) const;

  /** @returns the Center of Mass position, velocity and acceleration in 2D.
    *
    * @param t_global current time
    */
  StateLin2d GetCom(double t_global) const;
  double GetTotalTime() const;



  double GetZHeight() const { return com_spline_.z_height_; };



  /** @brief Calculates the Jacobian J of the motion with respect to the coefficients.
    *
    * @param t_global the time of the motion to evaluate the Jacobian
    * @param dxdt wheather Jacobian for position, velocity, acceleration or jerk is desired
    * @param dim which motion dimension (x,y) the jacobian represents.
    */
  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, Coords3D dim) const;
//  virtual JacobianRow GetJacobianVelSquared(double t_global, Coords3D dim) const = 0;
//  virtual JacobianRow GetJacobianPosVelSquared(double t_global, Coords3D dim) const = 0;


  /** Set all coefficients to fully describe the CoM motion.
    *
    * These can be spline coefficients or coefficients from any type of equation
    * that produce x(t) = ...
    */
//  virtual VectorXd GetXYSplineCoeffients() const = 0;

  // zmp_ make private
  ComSpline com_spline_;
protected:

//  virtual void SetSplineXYCoefficients(const VectorXd& coeff) = 0;


private:

};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_ */
