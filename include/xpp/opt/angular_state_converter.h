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

/** @brief Enables conversions between euler angles and angular velocities.
 *
 * Formulas taken from kindr:
 * http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
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
  using Mapping     = Jacobian; //Eigen::Matrix3d;

  AngularStateConverter ();
  AngularStateConverter (const OrientationVariables&);
  virtual ~AngularStateConverter ();


  AngularVel GetAngularVelocity(double t) const;
  AngularAcc GetAngularAcceleration(double t) const;

  Jacobian GetDerivOfAngAccWrtCoeff(double t) const;

private:
  OrientationVariables euler_;


  /** @brief maps euler rates zyx to angular velocities in world.
   */
  Mapping GetM(const EulerAngles& zyx) const;

  /** @brief time derivative of GetM()
   */
  Mapping GetMdot(const EulerAngles& zyx, const EulerRates& zyx_d) const;

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


  Mapping M_; ///< mapping between euler rates and angular velocities
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_ANGULAR_STATE_CONVERTER_H_ */
