/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_VARIABLES_ANGULAR_STATE_CONVERTER_H_
#define TOWR_VARIABLES_ANGULAR_STATE_CONVERTER_H_

#include <array>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "cartesian_dimensions.h"
#include "node_spline.h"

namespace towr {

/**
 * @brief Converts Euler angles and derivatives to angular quantities.
 *
 * Euler angles, their first derivatives (Euler rates) and their second
 * derivatives fully define the angular state of an rigid body in space. This
 * class provides equivalent formulation of these values, specifically:
 * @li Orientation as Rotation matrix or Quaternion.
 * @li Angular velocity (x,y,z)
 * @li Angular acceleration (x,y,z)
 *
 * Furthermore, the Euler angle spline is fully defined by a set of node values.
 * This class also gives the derivative of the above quantities with respect
 * to the values of these nodes.
 *
 * Formulas taken from kindr:
 * http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
 *
 * See matlab script "matlab/euler_converter.m" for derivation.
 */
class EulerConverter {
public:
  using Vector3d    = Eigen::Vector3d;
  using EulerAngles = Vector3d; ///< roll, pitch, yaw.
  using EulerRates  = Vector3d; ///< derivative of the above

  using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;
  using MatrixSXd   = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using Jacobian    = MatrixSXd;
  using JacRowMatrix = std::array<std::array<JacobianRow, k3D>, k3D>;

  EulerConverter () = default;

  /**
   * @brief Constructs and links this object to the Euler angle values
   * @param euler_angles_spline  The Euler angle spline defined by node values.
   *
   * The order of application that is assumed here is (Z,Y',X'').
   * So in order to get the orientation of an object in space from the Euler
   * angles, we first rotate around yaw(z) axis, then around new pitch(y)
   * axis and finally around new roll(x) axis.
   *
   * However, the 3-dimensional Euler angles must store the euler angles, rates, and
   * rate derivatives in the order (roll, pitch, yaw):
   * double roll  = euler_angles->GetPoint(t).x();
   * double pitch = euler_angles->GetPoint(t).y();
   * double yaw   = euler_angles->GetPoint(t).z();
   */
  EulerConverter (const NodeSpline::Ptr& euler_angles);
  virtual ~EulerConverter () = default;

  /**
   * @brief Converts the Euler angles at time t to a Quaternion.
   * @param t The current time in the euler angles spline.
   * @return A Quaternion the maps a vector from base to world frame.
   */
  Eigen::Quaterniond GetQuaternionBaseToWorld (double t) const;

  /**
   * @brief Converts the Euler angles at time t to a rotation matrix.
   * @param t The current time in the euler angles spline.
   * @return A 3x3 rotation matrix that maps a vector from base to world frame.
   */
  MatrixSXd GetRotationMatrixBaseToWorld(double t) const;

  /** @see GetRotationMatrixBaseToWorld(t)  */
  static MatrixSXd GetRotationMatrixBaseToWorld(const EulerAngles& xyz);

  /**
   * @brief Converts Euler angles and Euler rates to angular velocities.
   * @param t The current time in the euler angles spline.
   * @return A 3-dim vector (x,y,z) of the angular velocities in world frame.
   */
  Vector3d GetAngularVelocityInWorld(double t) const;

  /**
   * @brief Converts Euler angles, rates and rate derivatives  o angular accelerations.
   * @param t The current time in the euler angles spline.
   * @return A 3-dim vector (x,y,z) of the angular accelerations in world frame.
   */
  Vector3d GetAngularAccelerationInWorld(double t) const;

  /**
   * @brief Jacobian of the angular velocity with respect to the Euler nodes.
   * @param t  The current time in the Euler angles spline.
   * @return  The 3xn Jacobian Matrix of derivatives.
   *          3: because angular velocity has an x,y,z component.
   *          n: the number of optimized nodes values defining the Euler spline.
   */
  Jacobian GetDerivOfAngVelWrtEulerNodes(double t) const;

  /**
   * @brief Jacobian of the angular acceleration with respect to the Euler nodes.
   * @param t  The current time in the Euler angles spline.
   * @return The 3xn Jacobian Matrix of derivatives.
   *          3: because angular acceleration has an x,y,z component
   *          n: the number of optimized nodes values defining the Euler spline.
   */
  Jacobian GetDerivOfAngAccWrtEulerNodes(double t) const;

  /** @brief Returns the derivative of result of the linear equation M*v.
   *
   * M is the rotation matrix from base to world, defined by the Euler nodes.
   * v is any vector independent of the Euler nodes. The sensitivity
   * of the 3-dimensional vector w.r.t the Euler node values is given.
   *
   * @param t        time at which the Euler angles are evaluated.
   * @param v        vector (e.g. relative position, velocity, acceleration).
   * @param inverse  if true, the derivative for M^(-1)*v is evaluated.
   * @returns        3 x n dimensional matrix (n = number of Euler node values).
   */
  Jacobian DerivOfRotVecMult(double t, const Vector3d& v, bool inverse) const;

  /** @see GetQuaternionBaseToWorld(t)  */
  static Eigen::Quaterniond GetQuaternionBaseToWorld(const EulerAngles& pos);

private:
  NodeSpline::Ptr euler_;

  // Internal calculations for the conversion from euler rates to angular
  // velocities and accelerations. These are done using the matrix M defined
  // here: http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
  /**
   * @brief Matrix that maps euler rates to angular velocities in world.
   *
   * Make sure euler rates are ordered roll-pitch-yaw. They are however applied
   * in the order yaw-pitch-role to determine the angular velocities.
   */
  static MatrixSXd GetM(const EulerAngles& xyz);

  /**
   *  @brief time derivative of GetM()
   */
  static MatrixSXd GetMdot(const EulerAngles& xyz, const EulerRates& xyz_d);

  /**
   *  @brief Derivative of the @a dim row of matrix M with respect to
   *         the node values.
   *
   *  @param dim  Which dimension of the angular acceleration is desired.
   *  @returns    the Jacobian w.r.t the coefficients for each of the 3 rows
   *              of the matrix stacked on top of each other.
   */
  Jacobian GetDerivMwrtNodes(double t, Dim3D dim) const;

  /** @brief Derivative of the @a dim row of the time derivative of M with
   *         respect to the node values.
   *
   *  @param dim Which dimension of the angular acceleration is desired.
   */
  Jacobian GetDerivMdotwrtNodes(double t, Dim3D dim) const;

  /** @brief matrix of derivatives of each cell w.r.t node values.
   *
   * This 2d-array has the same dimensions as the rotation matrix M_IB, but
   * each cell if filled with a row vector.
   */
  JacRowMatrix GetDerivativeOfRotationMatrixWrtNodes(double t) const;

  /** @see GetAngularAccelerationInWorld(t)  */
  static Vector3d GetAngularAccelerationInWorld(State euler);

  /** @see GetAngularVelocityInWorld(t)  */
  static Vector3d GetAngularVelocityInWorld(const EulerAngles& pos,
                                            const EulerRates& vel);

  JacobianRow GetJac(double t, Dx deriv, Dim3D dim) const;
  Jacobian jac_wrt_nodes_structure_;
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_ANGULAR_STATE_CONVERTER_H_ */
