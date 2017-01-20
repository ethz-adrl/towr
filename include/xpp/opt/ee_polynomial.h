/**
 @file    ee_height_z_polynomial.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 16, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_POLYNOMIAL_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_POLYNOMIAL_H_

#include <xpp/utils/state.h>
#include <xpp/utils/polynomial_xd.h>
#include <xpp/utils/polynomial.h>

namespace xpp {
namespace opt {

class EEPolynomial {
public:
  using ZState     = xpp::utils::StateLin1d;
  using XYState    = xpp::utils::StateLin2d;
  using XYZState   = xpp::utils::StateLin3d;
  using PolyXY     = xpp::utils::PolynomialXd< utils::CubicPolynomial, XYState>;
  using PolyZ      = xpp::utils::QuinticPolynomial;

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
