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

#ifndef TOWR_MODELS_CENTROIDAL_MODEL_H_
#define TOWR_MODELS_CENTROIDAL_MODEL_H_

#include "dynamic_model.h"

namespace towr {

/**
 * @brief Centroidal Dynamics model relating forces to base accelerations.
 *
 * This class implements a Centroidal dynamics model, a reduced dimensional
 * model, lying in terms of accuracy between a simple Linear Inverted Pendulum
 * model and a complex full Rigid-body-dynamics Model.
 *
 * This model has the advantage that all required quantities are expressed
 * in Cartesian space, so Inverse Kinematics can be avoided.
 */
class CentroidalModel : public DynamicModel {
public:
  /**
   * @brief Constructs a specific Centroidal model.
   * @param mass         The mass of the robot.
   * @param ee_count     The number of endeffectors/forces.
   * @param W_inertia_W  The elements of the 3x3 Inertia matrix. This matrix
   *                     should map angular accelerations expressed in world frame
   *                     to Moments in world frame.
   */
  CentroidalModel (double mass, const Eigen::Matrix3d& W_inertia_W, int ee_count);

  /**
   * @brief Constructs a specific Centroidal model.
   * @param mass      Mass of the robot.
   * @param I..       Elements of the 3x3 Inertia matrix (@see CentroidalModel())
   * @param ee_count  Number of endeffectors/forces.
   */
  CentroidalModel (double mass,
                   double Ixx, double Iyy, double Izz,
                   double Ixy, double Ixz, double Iyz,
                   int ee_count);

  virtual ~CentroidalModel () = default;

  // for documentation, see definition in base class DynamicModel
  virtual BaseAcc GetBaseAccelerationInWorld() const override;

  virtual Jac GetJacobianOfAccWrtBaseLin(const Jac& jac_base_lin_pos) const override;
  virtual Jac GetJacobianOfAccWrtBaseAng(const Jac& jac_ang_vel) const override;
  virtual Jac GetJacobianofAccWrtForce(const Jac& jac_force, EE) const override;
  virtual Jac GetJacobianofAccWrtEEPos(const Jac& jac_ee_pos, EE) const override;

private:
  Eigen::Matrix3d I_dense_;                            // base inertia (dense)
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_;     // base inertia (sparse)
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_inv_; // inverse of base inertia
};


} /* namespace towr */

#endif /* TOWR_MODELS_CENTROIDAL_MODEL_H_ */
