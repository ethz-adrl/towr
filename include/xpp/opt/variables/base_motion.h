/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the ComMotion class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_

#include <memory>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/polynomial_spline.h>

namespace xpp {
namespace opt {

/** @brief Parameterizes the 6D (x,y,z,r,p,y) base of a system.
  *
  * This class is responsible for providing a common interface to represent
  * the motion of a system. Specific parametrizations can for example use
  * splines or solutions of the Equation of Motion as representation.
  */
class BaseMotion : public Composite {
public:
  using PolySplinePtr = std::shared_ptr<PolynomialSpline>;

  BaseMotion (const PolySplinePtr& linear, const PolySplinePtr& angular);
  virtual ~BaseMotion ();

  /** @brief Calculates the Jacobian J of the motion with respect to the coefficients.
    *
    * @param t_global the time of the motion to evaluate the Jacobian
    * @param dxdt wheather Jacobian for position, velocity, acceleration or jerk is desired
    * @param dim which motion dimension (x,y) the jacobian represents.
    */
  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, Coords6D dim) const;
  Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const;

  State3d GetBase(double t_global) const;

  double GetTotalTime() const;
  PolynomialSpline GetComSpline() const;

private:
  PolySplinePtr linear_; // to retain specific spline info
  PolySplinePtr angular_; // to retain specific spline info

  StateLin3d GetLinear(double t_global) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_ */
