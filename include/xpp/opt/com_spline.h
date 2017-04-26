/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_COM_SPLINE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_COM_SPLINE_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include "polynomial.h"
#include "polynomial_xd.h"
#include <xpp/opt/constraints/composite.h>

namespace xpp {
namespace opt {

/** Represents the Center of Mass (CoM) motion as a Spline (sequence of polynomials).
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a CoM position/velocity and acceleration.
  */
class ComSpline : public Component {
public:
  // zmp_ DRY, make state templated
  using PolyXdT        = PolynomialXd<CubicPolynomial, StateLin3d>;
  std::vector<Coords3D> dim_ = { X, Y , Z };
  using PolyHelpers    = PolyVecManipulation<PolyXdT>;
  using VecPolynomials = PolyHelpers::VecPolynomials;

  using Derivatives    = std::vector<MotionDerivative>;
  using PolyCoeff      = Polynomial::PolynomialCoeff;
  using JacobianRow    = Eigen::SparseVector<double, Eigen::RowMajor>;

  ComSpline ();
  virtual ~ComSpline ();

  void Init(double t_global, double duration_per_polynomial);

  VectorXd GetValues () const override;
  void SetValues (const VectorXd& optimized_coeff) override;

  StateLin3d GetCom(double t_global) const;
  double GetTotalTime() const;

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

  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, Coords3D dim) const;

private:
  VecPolynomials polynomials_;
  // careful: assumes all splines (X,Y,1,..,n) same type
  int NumFreeCoeffPerSpline() const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_COM_SPLINE_H_ */
