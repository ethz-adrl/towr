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

#include <xpp/opt/com_spline.h>
#include <xpp/opt/constraints/composite.h>

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
  using JacobianRow  = Eigen::SparseVector<double, Eigen::RowMajor>;
  using ComSplinePtr = std::shared_ptr<ComSpline>;

  BaseMotion (const ComSplinePtr&);
  virtual ~BaseMotion ();

  /** @brief Calculates the Jacobian J of the motion with respect to the coefficients.
    *
    * @param t_global the time of the motion to evaluate the Jacobian
    * @param dxdt wheather Jacobian for position, velocity, acceleration or jerk is desired
    * @param dim which motion dimension (x,y) the jacobian represents.
    */
  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, Coords6D dim) const;

  State3d GetBase(double t_global) const;
  StateLin3d GetCom(double t_global) const;

  double GetTotalTime() const;
  ComSpline GetComSpline() const;

//  void SetOffsetGeomToCom(const Vector3d&);

private:
  ComSplinePtr com_spline_; // to retain specific spline info
//  Vector3d offset_geom_to_com_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COM_MOTION_H_ */
