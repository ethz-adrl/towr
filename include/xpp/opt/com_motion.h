/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the ComMotion class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_

#include <xpp/matrix_vector.h>
#include <xpp/state.h>
#include <xpp/parametrization.h>

#include <Eigen/Sparse>
#include <memory>

namespace xpp {
namespace opt {

/** Abstracts the Center of Mass (CoM) motion of any system.
  *
  * This class is responsible for providing a common interface to represent
  * the motion of a system. Specific parametrizations can for example use
  * splines or solutions of the Equation of Motion as representation.
  */
class ComMotion : public Parametrization {
public:
  using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor> ;
  using PtrS        = std::shared_ptr<ComMotion> ;
  using PtrU        = std::unique_ptr<ComMotion> ;

  ComMotion ();
  virtual ~ComMotion ();


  VectorXd GetOptimizationParameters() const override;
  void SetOptimizationParameters(const VectorXd&) override;


  // zmp_ remove this, replace with GetOptVar()
  virtual int GetTotalFreeCoeff() const = 0;



  void SetOffsetGeomToCom(const Vector3d& offset);
  State3d GetBase(double t_global) const;

  /** @returns the Center of Mass position, velocity and acceleration in 2D.
    *
    * @param t_global current time
    */
  virtual StateLin2d GetCom(double t_global) const = 0;
  virtual double GetTotalTime() const = 0;



  /** @brief Calculates the Jacobian J of the motion with respect to the current coefficients.
    *
    * @param t_global the time of the motion to evaluate the Jacobian
    * @param dxdt wheather Jacobian for position, velocity, acceleration or jerk is desired
    * @param dim which motion dimension (x,y) the jacobian represents.
    */
  virtual JacobianRow GetJacobian(double t_global, MotionDerivative dxdt,
                                                   Coords3D dim) const = 0;

  virtual JacobianRow GetJacobianVelSquared(double t_global, Coords3D dim) const = 0;
  virtual JacobianRow GetJacobianPosVelSquared(double t_global, Coords3D dim) const = 0;

  /** @brief Creates a linear approximation of the motion at the current coefficients.
    *
    * Given some general nonlinear function x(u) = ... that represents the motion
    * of the system. A linear approximation of this function around specific
    * coefficients u* can be found using the Jacobian J evaluated at that point and
    * the value of the original function at *u:
    *
    * x(u) ~ J(u*)*(u-u*) + x(u*)
    *
    * @return The Jacobian J(u*) evaluated at u* and the corresponding offset x(u*).
    */
  VecScalar GetLinearApproxWrtCoeff(double t_global, MotionDerivative,
                                                     Coords3D dim) const;

  /** @brief Copies the derived class in the heap and returns a pointer to it.
    */
  virtual PtrU clone() const = 0;

  double GetZHeight() const { return z_height_; };
  void SetConstantHeight(double z) { z_height_ = z; };


  /** Set all coefficients to fully describe the CoM motion.
    *
    * These can be spline coefficients or coefficients from any type of equation
    * that produce x(t) = ...
    */
  // zmp_ rename to getXY spline coefficients
  virtual VectorXd GetCoeffients() const = 0;

protected:
  virtual void SetSplineXYCoefficients(const VectorXd& coeff) = 0;


private:
  double z_height_;
  Vector3d offset_geom_to_com_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_ */
