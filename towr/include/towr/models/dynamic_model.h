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

#ifndef TOWR_MODELS_DYNAMIC_MODEL_H_
#define TOWR_MODELS_DYNAMIC_MODEL_H_

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <towr/variables/euler_converter.h>

namespace towr {

/**
 * @brief A interface for the the system dynamics of a legged robot.
 *
 * This class is responsible for verifying that the current acceleration of a
 * system given a specific robot state and input forces ensure the system
 * dynamics, so
 * g = xdd(t) - f(x(t), f(t)).
 *
 * Dynamic models can for example be:
 * Linear Inverted Pendulum (LIP),
 * SingleRigidBodyDynamics (SRBD),
 * Centroidal Dynamics,
 * Full Rigid Body Dynamics (RBD).
 *
 * An overview of all these models can be found here:
 * https://doi.org/10.3929/ethz-b-000272432
 *
 * This model is used in DynamicConstraint to ensure that the optimized
 * motion trajectory complies to this. Currently, only SingleRigidBodyDynamics
 * is implemented.
 *
 * @ingroup Robots
 */
class DynamicModel {
public:
  using Ptr      = std::shared_ptr<DynamicModel>;
  using Vector3d = Eigen::Vector3d;
  using Matrix3d = Eigen::Matrix3d;
  using ComPos   = Eigen::Vector3d;
  using AngVel   = Eigen::Vector3d;
  using BaseAcc  = Eigen::Matrix<double,6,1>;
  using Jac      = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using EEPos    = std::vector<Eigen::Vector3d>;
  using EELoad   = EEPos;
  using EE       = uint;

  /**
   * @brief Sets the current state and input of the system.
   * @param com_W        Current Center-of-Mass (x,y,z) position in world frame.
   * @param com_acc_W    Current Center-of-Mass (x,y,z) acceleration in world.
   * @param w_R_b        Current rotation from base to world frame.
   * @param omega_W      Current angular velocity in world frame.
   * @param omega_dot_W  Current angular acceleration in world frame.
   * @param force_W      Force at each foot expressed in world frame.
   * @param pos_W        Position of each foot expressed in world frame
   */
  void SetCurrent(const ComPos& com_W, const Vector3d com_acc_W,
                  const Matrix3d& w_R_b, const AngVel& omega_W, const Vector3d& omega_dot_W,
                  const EELoad& force_W, const EEPos& pos_W);

  /**
   * @brief  The violation of the system dynamics incurred by the current values.
   * @return The 6-dimension generalized force violation (angular + linear).
   */
  virtual BaseAcc GetDynamicViolation() const = 0;

  /**
   * @brief How the base position affects the dynamic violation.
   * @param jac_base_lin_pos  The 3xn Jacobian of the base linear position.
   * @param jac_base_lin_acc  The 3xn Jacobian of the base linear acceleration.
   *
   * @return The 6xn Jacobian of dynamic violations with respect to
   *         variables defining the base linear spline (e.g. node values).
   */
  virtual Jac GetJacobianWrtBaseLin(const Jac& jac_base_lin_pos,
                                    const Jac& jac_base_lin_acc) const = 0;

  /**
   * @brief How the base orientation affects the dynamic violation.
   * @param base_angular  provides Euler angles Jacobians.
   * @param t  Time at which euler angles values are queried.
   *
   * @return The 6xn Jacobian of dynamic violations with respect to
   *         variables defining the base angular spline (e.g. node values).
   */
  virtual Jac GetJacobianWrtBaseAng(const EulerConverter& base_angular,
                                    double t) const = 0;

  /**
   * @brief How the endeffector forces affect the dynamic violation.
   * @param ee_force  The 3xn Jacobian of the foot force x,y,z.
   * @param ee        The endeffector for which the senstivity is required.
   *
   * @return The 6xn Jacobian of dynamic violations with respect to
   *         variables defining the endeffector forces (e.g. node values).
   */
  virtual Jac GetJacobianWrtForce(const Jac& ee_force, EE ee) const = 0;

  /**
   * @brief How the endeffector positions affect the dynamic violation.
   * @param ee_force  The 3xn Jacobian of the foot position x,y,z.
   * @param ee        The endeffector for which the senstivity is required.
   *
   * @return The 6xn Jacobian of dynamic violations with respect to
   *         variables defining the foot positions (e.g. node values).
   */
  virtual Jac GetJacobianWrtEEPos(const Jac& ee_pos, EE ee) const = 0;

  /**
   * @returns The gravity acceleration [m/s^2] (positive)
   */
  double g() const { return g_; };

  /**
   * @returns The mass of the robot [kg].
   */
  double m() const { return m_; };

  /**
   * @brief the number of endeffectors that this robot has.
   */
  int GetEECount() const { return ee_pos_.size(); };

protected:
  ComPos com_pos_;   ///< x-y-z position of the Center-of-Mass.
  Vector3d com_acc_; ///< x-y-z acceleration of the Center-of-Mass.

  Matrix3d w_R_b_;     ///< rotation matrix from base (b) to world (w) frame.
  AngVel omega_;       ///< angular velocity expressed in world frame.
  Vector3d omega_dot_; ///< angular acceleration expressed in world frame.

  EEPos  ee_pos_;   ///< The x-y-z position of each endeffector.
  EELoad ee_force_; ///< The endeffector force expressed in world frame.

  /**
   * @brief Construct a dynamic object. Protected as this is abstract base class.
   * @param mass The mass of the system.
   */
  DynamicModel(double mass, int ee_count);
  virtual ~DynamicModel () = default;

private:
  double g_; ///< gravity acceleration [m/s^2]
  double m_; ///< mass of the robot
};

} /* namespace towr */

#endif /* TOWR_MODELS_DYNAMIC_MODEL_H_ */
