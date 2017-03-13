/**
 @file    ee_height_z_polynomial.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 16, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_POLYNOMIAL_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_POLYNOMIAL_H_

#include "polynomial_xd.h"
#include "polynomial.h"
#include <xpp/state.h>

namespace xpp {
namespace opt {

class EEPolynomial {
public:
  using ZState     = StateLin1d;
  using XYState    = StateLin2d;
  using XYZState   = StateLin3d;
  using PolyXY     = PolynomialXd<CubicPolynomial, XYState>;
  using PolyZ      = QuinticPolynomial;

  EEPolynomial ();
  virtual ~EEPolynomial ();


  void SetDuration(double T);

  // assume these values don't change, otherwise second function wrong
  void SetZParams(double percent_done, double z_max);
  void SetXYParams(const XYState& start, const XYState& end);

  XYZState GetState(double t_local) const;

private:
  PolyZ poly_z_;
  PolyXY poly_xy_;

  double t_start_z_;
  double duration_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_POLYNOMIAL_H_ */
