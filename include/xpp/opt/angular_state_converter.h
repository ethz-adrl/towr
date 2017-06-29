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

/** @brief Provides conversions from euler angles.
 *
 * If euler angles are to be given, they must be ordered in the vector by
 * x,y,z (roll first element, pitch second, yaw third).
 *
 * Formulas taken from kindr:
 * http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
 *
 * See matlab script "matlab/euler_angular_conversions.m" for derivation.
 */
class AngularStateConverter {
public:
  using OrientationVariables = std::shared_ptr<PolynomialSpline>;
  using EulerAngles = Vector3d; ///< order x,y,z (roll, pitch, yaw).
  using EulerRates  = Vector3d; ///< derivative of the above
  using AngularVel  = Vector3d; ///< expressed in world
  using AngularAcc  = Vector3d; ///< expressed in world

  AngularStateConverter ();
  AngularStateConverter (const OrientationVariables&);
  virtual ~AngularStateConverter ();

  static StateAng3d GetState(const StateLin3d& euler);

  static Eigen::Quaterniond GetOrientation(const EulerAngles& pos);

  AngularVel GetAngularVelocity(double t) const;
  static AngularVel GetAngularVelocity(const EulerAngles& pos, const EulerAngles& vel);

  static AngularAcc GetAngularAcceleration(StateLin3d euler);
  AngularAcc GetAngularAcceleration(double t) const;

  Jacobian GetDerivOfAngAccWrtCoeff(double t) const;


  /** @returns the rotations matrix that rotates a vector
   *
   * This rotation matrix expresses a vector previously given in base frame
   * in the world frame. The euler angles are applied in the order zyx, but
   * must still be passed in as vector (x,y,z).
   */
  static MatrixSXd GetRotationMatrixBaseToWorld(const EulerAngles& xyz);
  MatrixSXd GetRotationMatrixBaseToWorld(double t) const;

  /** @brief Returns the derivative of the linear equation M*v.
   *
   * M is the rotation matrix from base to world M_IB, and v a 3-dimensional
   * vector expressed in the base frame.
   *
   * @param t        time at which the euler angles are evaluated.
   * @param v        vector (e.g. relative position) expressed in base frame.
   * @param inverse  if true, the derivative for M^(-1)*v is evaluated.
   * @returns        3 x n dimensional matrix (n = number of spline parameters).
   */
  Jacobian GetDerivativeOfRotationMatrixRowWrtCoeff(double t,
                                                    const Vector3d& v,
                                                    bool inverse) const;

private:
  OrientationVariables euler_;

  /// Internal calculations for the conversion from euler rates to angular
  /// velocities and accelerations. These are done using the matrix M defined
  /// here: http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf

  /** @brief Matrix that maps euler rates to angular velocities in world.
   *
   * Make sure euler rates are ordered roll-pitch-yaw. They are however applied
   * in the order yaw-pitch-role to determine the angular velocities.
   */
  static MatrixSXd GetM(const EulerAngles& xyz);

  /** @brief time derivative of GetM()
   */
  static MatrixSXd GetMdot(const EulerAngles& xyz, const EulerRates& xyz_d);

  /** @brief Derivative of the @a dim row of matrix M with respect to
   *         the polynomial coefficients.
   *
   *  @param dim  Which dimension of the angular acceleration is desired.
   *  @returns    the Jacobians w.r.t the coefficients for each of the 3 rows
   *              of the matrix stacked on top of each other.
   */
  Jacobian GetDerivMwrtCoeff(double t, Coords3D dim) const;

  /** @brief Derivative of the @a dim row of the time derivative of M with
   *         respect to the polynomial coefficients.
   *
   *  @param dim Which dimension of the angular acceleration is desired
   */
  Jacobian GetDerivMdotwrtCoeff(double t, Coords3D dim) const;

  using JacRowMatrix = std::array<std::array<JacobianRow, kDim3d>, kDim3d>;
  /** @brief matrix of derivatives of each cell w.r.t spline coefficients
   *
   * This 2d-array has the same dimensions as the rotation matrix M_IB, but
   * each cell if filled with a row vector.
   */
  JacRowMatrix GetDerivativeOfRotationMatrixWrtCoeff(double t) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_ANGULAR_STATE_CONVERTER_H_ */
