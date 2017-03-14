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

class EESwingMotion {
public:
  using PolyXY     = PolynomialXd<CubicPolynomial, StateLin2d>;
  using PolyZ      = QuinticPolynomial;

  EESwingMotion ();
  virtual ~EESwingMotion ();


  void SetDuration(double T);

  // assume these values don't change, otherwise second function wrong

  // zmp_ remove this "percent_done", seems antique
  //zmp_ this should also allow 3D foothold
//  void SetZParams(double percent_done, double z_max);
  void SetContacts(const Vector3d& start, const Vector3d& end);

  StateLin3d GetState(double t_local) const;

private:
  PolyZ poly_z_;
  PolyXY poly_xy_;

//  double t_start_z_; // zmp_ remove this crap
  double duration_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_SWING_MOTION_H_ */
