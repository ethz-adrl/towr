/**
 @file    ee_height_z_polynomial.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 16, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_SWING_MOTION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_SWING_MOTION_H_

#include "polynomial_xd.h"
#include "polynomial.h"
#include <xpp/state.h>

namespace xpp {
namespace opt {

/** Parametrizes the motion from one 3D point to another.
  *
  * This can be used to generate the swingleg motion given two footholds.
  * See xpp_opt/matlab/swingleg_z_height.m for the generation of these values.
  */
class EESwingMotion {
public:
  using PolyXY     = PolynomialXd<CubicPolynomial, StateLin2d>;
  using PolyZ      = QuinticPolynomial;

  EESwingMotion ();
  virtual ~EESwingMotion ();

  /** The duration[s] of the motion.
    */
  void SetDuration(double T);

  /** Determines how high the leg is lifted between contact points.
    *
    * This is not the height exactly between the contact points, but the height
    * that the swingleg has as 1/n_*T and (n-1)/n*T, e.g. shortly after lift-off
    * and right before touchdown. The lift-height in the center is higher.
    */
  void SetLiftHeight(double h);

  /** @param start The xyz starting location.
    * @param end The goal of the motion reached after T.
    */
  void SetContacts(const Vector3d& start, const Vector3d& end);

  /** Returns the 3D position, velocity and acceleration at any time during
    * the motion.
    */
  StateLin3d GetState(double t_local) const;
  double GetDuration() const;

private:
  PolyZ poly_z_;
  PolyXY poly_xy_;

  double T_;        ///< the duration [s] of the motion
  double h_ = 0.03; ///< proportional to the lift height between contacts
  int n_ = 6;       ///< determines the shape of the swing motion
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_SWING_MOTION_H_ */
