/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_VARIABLES_ANGULAR_STATE_CONVERTER_H_
#define TOWR_VARIABLES_ANGULAR_STATE_CONVERTER_H_

#include <array>

#include "cartesian_dimensions.h"
#include "spline.h"

namespace towr {

/** @brief Provides conversions from euler angles.
 *
 * If euler angles are to be given, they must be ordered in the vector by
 * x,y,z (roll first element, pitch second, yaw third). They represent
 * the ZY'X'' convention, meaning they first rotated around z-axis by yaw,
 * then around new y-axis by pitch, finally around newest x-axis by roll.
 *
 * Formulas taken from kindr:
 * http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
 *
 * See matlab script "matlab/euler_angular_conversions.m" for derivation.
 */
class AngularStateConverter {
public:
  using Vector3d    = Eigen::Vector3d;
  using EulerAngles = Vector3d; ///< order x,y,z (roll, pitch, yaw).
  using EulerRates  = Vector3d; ///< derivative of the above
  using AngularVel  = Vector3d; ///< expressed in world
  using AngularAcc  = Vector3d; ///< expressed in world


  using Jacobian    = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using MatrixSXd   = Jacobian;
  using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;
  using StateLin3d  = State;

  AngularStateConverter () = default;
  AngularStateConverter (const Spline::Ptr&);
  virtual ~AngularStateConverter () = default;


  static Eigen::Quaterniond GetOrientation(const EulerAngles& pos);

  AngularVel GetAngularVelocity(double t) const;
  static AngularVel GetAngularVelocity(const EulerAngles& pos, const EulerAngles& vel);
  Jacobian GetDerivOfAngVelWrtCoeff(double t) const;

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
  Spline::Ptr euler_;

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
  Jacobian GetDerivMwrtCoeff(double t, Dim3D dim) const;

  /** @brief Derivative of the @a dim row of the time derivative of M with
   *         respect to the polynomial coefficients.
   *
   *  @param dim Which dimension of the angular acceleration is desired
   */
  Jacobian GetDerivMdotwrtCoeff(double t, Dim3D dim) const;

  using JacRowMatrix = std::array<std::array<JacobianRow, k3D>, k3D>;
  /** @brief matrix of derivatives of each cell w.r.t spline coefficients
   *
   * This 2d-array has the same dimensions as the rotation matrix M_IB, but
   * each cell if filled with a row vector.
   */
  JacRowMatrix GetDerivativeOfRotationMatrixWrtCoeff(double t) const;

  /** number of optimization variables of polynomial active at time t.
   *
   * usually these are the polynomial coefficients, but for e.g. cubic
   * hermite spline these might be the node values and derivatives.
   */
  int OptVariablesOfCurrentPolyCount(double t) const;

  JacobianRow GetJac(double t, Dx deriv, Dim3D dim) const;
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_ANGULAR_STATE_CONVERTER_H_ */
