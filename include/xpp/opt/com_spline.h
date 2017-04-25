/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/matrix_vector.h>

#include "polynomial.h"
#include "polynomial_xd.h"

namespace xpp {
namespace opt {

/** Represents the Center of Mass (CoM) motion as a Spline (sequence of polynomials).
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a CoM position/velocity and acceleration.
  */
// zmp_ make this a member variable of base motion, not deriving from it!
class ComSpline {
public:
  using Derivatives    = std::vector<MotionDerivative>;
  using PolyCoeff      = Polynomial::PolynomialCoeff;
  using PolyXdT        = PolynomialXd<CubicPolynomial, StateLin2d>;
  using PolyHelpers    = PolyVecManipulation<PolyXdT>;
  using VecPolynomials = PolyHelpers::VecPolynomials;
  using JacobianRow    = Eigen::SparseVector<double, Eigen::RowMajor>;

  ComSpline ();
  virtual ~ComSpline ();

  void Init(double t_global, double duration_per_polynomial);

  StateLin2d GetCom(double t_global) const;
  double GetTotalTime() const;
  VectorXd GetXYSplineCoeffients () const;
  int GetTotalFreeCoeff() const;

  int Index(int polynomial, Coords3D dim, PolyCoeff coeff) const;

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
  VecScalar GetLinearApproxWrtCoeff(double t_global, MotionDerivative, Coords3D dim) const;

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
  void SetSplineXYCoefficients (const VectorXd& optimized_coeff);
//  void SetCoefficientsZero();

  // zmp_ make private
  Vector3d offset_geom_to_com_;
  double z_height_ = 0.0;
  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, Coords3D dim) const;
private:
  VecPolynomials polynomials_;

  // careful: assumes all splines (X,Y,1,..,n) same type
  int NumFreeCoeffPerSpline() const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_ */
