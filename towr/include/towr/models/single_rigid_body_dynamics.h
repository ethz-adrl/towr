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

#ifndef TOWR_MODELS_SINGLE_RIBID_BODY_DYNAMICS_MODEL_H_
#define TOWR_MODELS_SINGLE_RIBID_BODY_DYNAMICS_MODEL_H_

#include "dynamic_model.h"

namespace towr {

/**
 * @brief Dynamics model relating forces to base accelerations.
 *
 * This class implements a Single Rigid Body dynamics model, a reduced
 * dimensional model, lying in terms of accuracy between a Linear
 * Inverted Pendulum model and a full Centroidal or Rigid-body-dynamics model.
 *
 * This model makes the assumption that the motion of the limbs does not
 * incur significant momentum and can therefore be neglected. This eliminates
 * the nonlinear dependency on joint angles and allows to express
 * all quantities in Cartesian space.
 *
 * For the derivation and all assumptions of this model, see:
 * https://doi.org/10.3929/ethz-b-000272432
 *
 * \sa https://en.wikipedia.org/wiki/Newton%E2%80%93Euler_equations
 *
 * @ingroup Robots
 */
class SingleRigidBodyDynamics : public DynamicModel {
public:
  /**
   * @brief Constructs a specific model.
   * @param mass         The mass of the robot.
   * @param ee_count     The number of endeffectors/forces.
   * @param inertia_b    The elements of the 3x3 Inertia matrix around the CoM.
   *                     This matrix maps angular accelerations expressed in
   *                     base frame to moments in base frame.
   */
  SingleRigidBodyDynamics (double mass, const Eigen::Matrix3d& inertia_b, int ee_count);

  /**
   * @brief Constructs a specific model.
   * @param mass      Mass of the robot.
   * @param I..       Elements of the 3x3 Inertia matrix
   * @param ee_count  Number of endeffectors/forces.
   */
  SingleRigidBodyDynamics (double mass,
                   double Ixx, double Iyy, double Izz,
                   double Ixy, double Ixz, double Iyz,
                   int ee_count);

  virtual ~SingleRigidBodyDynamics () = default;

  BaseAcc GetDynamicViolation() const override;

  Jac GetJacobianWrtBaseLin(const Jac& jac_base_lin_pos,
                            const Jac& jac_acc_base_lin) const override;
  Jac GetJacobianWrtBaseAng(const EulerConverter& base_angular,
                            double t) const override;
  Jac GetJacobianWrtForce(const Jac& jac_force, EE) const override;

  Jac GetJacobianWrtEEPos(const Jac& jac_ee_pos, EE) const override;

private:
  /** Inertia of entire robot around the CoM expressed in a frame anchored
   *  in the base.
   */
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_b;
};


} /* namespace towr */

#endif /* TOWR_MODELS_SINGLE_RIBID_BODY_DYNAMICS_MODEL_H_ */
