/**
 @file    angular_state_converter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 27, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_ANGULAR_STATE_CONVERTER_H_
#define XPP_OPT_INCLUDE_XPP_OPT_ANGULAR_STATE_CONVERTER_H_

#include <memory>

#include "polynomial_spline.h"

namespace xpp {
namespace opt {

/** @brief Provides conversions between euler angles and angular velocities.
 *
 * Formulas taken from kindr:
 * http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
 *
 * Using Euler angles zyx convention (first rotate around yaw, then pitch, then roll).
 *
 * See matlab script "matlab/euler_angular_conversions.m" for derivation.
 */
class AngularStateConverter {
public:
  using OrientationVariables = std::shared_ptr<PolynomialSpline>;
  using EulerAngles = Vector3d;
  using EulerRates  = Vector3d;
  using AngularVel  = Vector3d;
  using AngularAcc  = Vector3d;

  AngularStateConverter ();
  AngularStateConverter (const OrientationVariables&);
  virtual ~AngularStateConverter ();

  AngularVel GetAngularVelocity(double t) const;
  AngularAcc GetAngularAcceleration(double t) const;

  Jacobian GetDerivOfAngAccWrtCoeff(double t) const;

private:
  OrientationVariables euler_;

  /** @brief maps euler rates zyx to angular velocities in world.
   *
   * Make sure euler rates are ordered roll-pitch-yaw.
   */
  Jacobian GetM(const EulerAngles& xyz) const;

  /** @brief time derivative of GetM()
   */
  Jacobian GetMdot(const EulerAngles& xyz, const EulerRates& xyz_d) const;

  /** @brief Derivative of the @a dim row of matrix M with respect to
   *         the polynomial coefficients.
   *
   *  @param dim Which dimension of the angular acceleration is desired
   */
  Jacobian GetDerivMwrtCoeff(double t, Coords3D dim) const;


  /** @brief Derivative of the @a dim row of the time derivative of M with
   *         respect to the polynomial coefficients.
   *
   *  @param dim Which dimension of the angular acceleration is desired
   */
  Jacobian GetDerivMdotwrtCoeff(double t, Coords3D dim) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_ANGULAR_STATE_CONVERTER_H_ */
