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


namespace towr {

/**
 * @brief A interface for the the system dynamics of a legged robot.
 *
 * This class is responsible for providing the current acceleration of a
 * system given a specific robot state and input forces, so:
 * xdd(t) = f(x(t), f(t)).
 *
 * This model is used in @ref DynamicConstraint to ensure that the optimized
 * motion trajectory complies to this.
 *
 * Currently, only @ref CentroidalModel is implemented, but this can be
 * extended in the future to also incorporate full rigid body dynamics. This
 * interface to the solver should remain the same.
 */
class DynamicModel {
public:
  using Ptr      = std::shared_ptr<DynamicModel>;
  using Vector3d = Eigen::Vector3d;
  using ComPos   = Eigen::Vector3d;
  using AngVel   = Eigen::Vector3d;
  using BaseAcc  = Eigen::Matrix<double,6,1>;
  using Jac      = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using EEPos    = std::vector<Eigen::Vector3d>;
  using EELoad   = EEPos;
  using EE       = uint;

  /**
   * @brief Sets the current state and input of the system.
   * @param com_W    The current Center-of-Mass (x,y,z) position.
   * @param omega_W  The current angular velocity in world frame.
   * @param force_W  The force at each foot expressed in world frame.
   * @param pos_W    The position of each foot expressed in world frame
   */
  void SetCurrent(const ComPos& com_W, const AngVel& omega_W,
                  const EELoad& force_W, const EEPos& pos_W);

  /**
   * @brief  The acceleration as defined by the system dynamics.
   * @return The 6-dimension accelerations (angular + linear) in World frame.
   */
  virtual BaseAcc GetBaseAccelerationInWorld() const = 0;

  /**
   * @brief How the base position affects the base acceleration.
   * @param jac_base_lin_pos  The 3xn Jacobian of the base linear position.
   *
   * @return The 6xn Jacobian of base acceleration with respect to
   *         variables defining the base linear spline (e.g. node values).
   */
  virtual Jac GetJacobianOfAccWrtBaseLin(const Jac& jac_base_lin_pos) const = 0;

  /**
   * @brief How the base orientation affects the base acceleration.
   * @param jac_base_ang_pos  The 3xn Jacobian of the base angular position.
   *
   * @return The 6xn Jacobian of base acceleration with respect to
   *         variables defining the base angular spline (e.g. node values).
   */
  virtual Jac GetJacobianOfAccWrtBaseAng(const Jac& jac_base_ang_pos) const = 0;

  /**
   * @brief How the endeffector forces affect the base acceleration.
   * @param ee_force  The 3xn Jacobian of the foot force x,y,z.
   * @param ee        The endeffector for which the senstivity is required.
   *
   * @return The 6xn Jacobian of base acceleration with respect to
   *         variables defining the endeffector forces (e.g. node values).
   */
  virtual Jac GetJacobianofAccWrtForce(const Jac& ee_force, EE ee) const = 0;

  /**
   * @brief How the endeffector positions affect the base acceleration.
   * @param ee_force  The 3xn Jacobian of the foot position x,y,z.
   * @param ee        The endeffector for which the senstivity is required.
   *
   * @return The 6xn Jacobian of base acceleration with respect to
   *         variables defining the foot positions (e.g. node values).
   */
  virtual Jac GetJacobianofAccWrtEEPos(const Jac& ee_pos, EE ee) const = 0;

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
  EEPos  ee_pos_;  ///< The x-y-z position of each endeffector.
  ComPos com_pos_; ///< The x-y-z position of the Center-of-Mass.
  AngVel omega_;   ///< The angular velocity expressed in world frame.
  EELoad ee_force_;///< The endeffector force expressed in world frame.

  /**
   * @brief Construct a dynamic object. Protected as this is abstract base class.
   * @param mass The mass of the system.
   */
  DynamicModel(double mass);
  virtual ~DynamicModel () = default;

private:
  double g_; ///< gravity acceleration [m/s^2]
  double m_; ///< mass of the robot
};

} /* namespace towr */

#endif /* TOWR_MODELS_DYNAMIC_MODEL_H_ */
