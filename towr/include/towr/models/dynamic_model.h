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

#ifndef TOWR_MODELS_DYNAMIC_MODEL_H_
#define TOWR_MODELS_DYNAMIC_MODEL_H_

#include <memory>
#include <vector>

#include <Eigen/Eigen>


namespace towr {

class DynamicModel {
public:
  using Ptr      = std::shared_ptr<DynamicModel>;
  using Vector3d = Eigen::Vector3d;
  using ComPos   = Eigen::Vector3d;
  using AngVel   = Eigen::Vector3d;
  using BaseAcc  = Eigen::Matrix<double,6,1>;
  using EE       = uint;
  using EEPos    = std::vector<Eigen::Vector3d>;
  using EELoad   = EEPos;
  using Jac      = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  DynamicModel(double mass);
  virtual ~DynamicModel () = default;

  void SetCurrent(const ComPos& com, const AngVel& w, const EELoad&, const EEPos&);

  virtual BaseAcc GetBaseAcceleration() const = 0;

  virtual Jac GetJacobianOfAccWrtBaseLin(const Jac& jac_base_lin_pos) const = 0;
  virtual Jac GetJacobianOfAccWrtBaseAng(const Jac& jac_base_ang_pos) const = 0;
  virtual Jac GetJacobianofAccWrtForce(const Jac& ee_force, EE) const = 0;
  virtual Jac GetJacobianofAccWrtEEPos(const Jac&, EE) const = 0;


  double GetGravityAcceleration() const { return g_; };
  double GetMass() const { return m_; };

protected:
  EEPos  ee_pos_;
  ComPos com_pos_;
  AngVel omega_;
  EELoad ee_force_;

  double g_; // gravity acceleration [m/s^2]
  double m_; // mass of the robot
};

} /* namespace towr */

#endif /* TOWR_MODELS_DYNAMIC_MODEL_H_ */
