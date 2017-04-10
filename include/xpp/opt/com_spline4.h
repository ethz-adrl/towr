/**
@file    com_spline4.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   Declares ComSpline4, which realizes a ComSpline
 */

#ifndef _XPP_ZMP_COMSPLINE4_H_
#define _XPP_ZMP_COMSPLINE4_H_

#include "com_spline.h"

namespace xpp {
namespace opt {

/** Represents the center of mass motion with 4 coefficients per polynomial
  *
  * This class represents a collection of fifth order polynomials
  * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f, that are
  * continuous in position and velocity at their borders. This means that the e
  * and f coefficients of all splines can be uniquely determined from the other
  * coefficients and the initial position/velocity.
  */
class ComSpline4 : public ComSpline {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef Eigen::Vector2d Vector2d;
  typedef Eigen::VectorXd VectorXd;
  using PtrClone = BaseMotion::PtrU;
  typedef Eigen::SparseMatrix<double, Eigen::RowMajor> JacobianEFWrtABCD;

  ComSpline4 ();
  virtual ~ComSpline4 ();

  void SetStartPosVel (const Vector2d& start_cog_p, const Vector2d& start_cog_v);

  void SetSplineXYCoefficients(const VectorXd& optimized_coeff) override;
  void SetEndAtStart();

  Derivatives GetInitialFreeMotions()  const;
  Derivatives GetJunctionFreeMotions() const;

private:
  int NumFreeCoeffPerSpline() const override { return 4; };
  std::vector<PolyCoeff> GetFreeCoeffPerSpline() const override
  {
    return {PolyCoeff::A,PolyCoeff::B,PolyCoeff::C,PolyCoeff::D};
  };

  void GetJacobianPos (double t_poly, int id, Coords3D dim, JacobianRow&) const override;
  void GetJacobianVel (double t_poly, int id, Coords3D dim, JacobianRow&) const override;
  void GetJacobianAcc (double t_poly, int id, Coords3D dim, JacobianRow&) const override;
  void GetJacobianJerk(double t_poly, int id, Coords3D dim, JacobianRow&) const override;

  /** Returns the Jacobian of coefficient E with respect to coefficients a,b,c,d
   *  for a specific spline k and dimension (x,y).
   */
  JacobianRow GetJacobianE(int spline_id_k, Coords3D dim) const;
  JacobianRow GetJacobianF(int spline_id_k, Coords3D dim) const;

  JacobianEFWrtABCD CalcJacobianEWrtABCD(Coords3D) const;
  JacobianEFWrtABCD CalcJacobianFWrtABCD(Coords3D) const;
  std::array<JacobianEFWrtABCD, kDim2d> jac_e_wrt_abcd_;
  std::array<JacobianEFWrtABCD, kDim2d> jac_f_wrt_abcd_;

  Vector2d start_cog_p_;
  Vector2d start_cog_v_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif // _XPP_ZMP_COMSPLINE4_H_
