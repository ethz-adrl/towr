/**
 @file    ee_phase_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 16, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_SWING_MOTION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_SWING_MOTION_H_

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <../../xpp_opt/polynomial.h>

namespace xpp {
namespace opt {

/** Parametrizes the motion from one 3D point to another.
  *
  * This can be used to generate the swingleg motion given two footholds or
  * represent a leg in stance ("movement" between the same start/goal).
  */
class EEPhaseMotion : public Segment {
public:
  using PolyXY = CubicPolynomial;
  using PolyZ  = LiftHeightPolynomial;

  EEPhaseMotion ();
  virtual ~EEPhaseMotion ();

  /** Completely parametrizes the motion.
    *
    * h is not the exact height between the contact points, but the height
    * that the swingleg has as 1/n_*T and (n-1)/n*T, e.g. shortly after lift-off
    * and right before touchdown. The lift-height in the center is higher.
    *
    * @param T The duration[s] of the motion.
    * @param h Determines how high the leg is lifted between contact points.
    * @param start The xyz starting location.
    * @param end The goal of the motion reached after T.
    */
  void Init(double T, double h, const Vector3d& start, const Vector3d& end);

  /** @param start The xyz starting location.
    * @param end The goal of the motion reached after T.
    */
  void SetContacts(const Vector3d& start, const Vector3d& end);

  /** Returns the 3D position, velocity and acceleration at any time during
    * the motion.
    */
  StateLinXd GetPoint(double t_local) const;
  double GetDuration() const;

  double GetDerivativeOfPosWrtPos(double t_local, Polynomial::PointType p) const;

  PolyXY poly_xy_;
private:
  PolyZ poly_z_;
  double T_ = 0.0;   ///< the duration [s] of the motion
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_SWING_MOTION_H_ */
