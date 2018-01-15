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

#ifndef TOWR_MODELS_CENTROIDAL_MODEL_H_
#define TOWR_MODELS_CENTROIDAL_MODEL_H_

#include "dynamic_model.h"

namespace towr {

/**
 * @brief Centroidal Dynamics for the 6-DoF Base to model the system.
 */
class CentroidalModel : public DynamicModel {
public:
  CentroidalModel (double mass,
                   double Ixx, double Iyy, double Izz,
                   double Ixy, double Ixz, double Iyz,
                   int ee_count);
  CentroidalModel (double mass, const Eigen::Matrix3d& inertia, int ee_count);
  virtual ~CentroidalModel () = default;

  virtual BaseAcc GetBaseAcceleration() const override;

  virtual Jac GetJacobianOfAccWrtBaseLin(const Jac& jac_base_lin_pos) const override;
  virtual Jac GetJacobianOfAccWrtBaseAng(const Jac& jac_ang_vel) const override;
  virtual Jac GetJacobianofAccWrtForce(const Jac& jac_force, EE) const override;
  virtual Jac GetJacobianofAccWrtEEPos(const Jac& jac_ee_pos, EE) const override;

private:
  Eigen::Matrix3d I_dense_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_; // inverse of base inertia
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_inv_; // inverse of base inertia
};


} /* namespace towr */

#endif /* TOWR_MODELS_CENTROIDAL_MODEL_H_ */
