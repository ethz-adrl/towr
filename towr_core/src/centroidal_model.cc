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

#include <towr/models/centroidal_model.h>
#include <towr/variables/cartesian_dimensions.h>

namespace towr {

// some Eigen helper functions
static Eigen::Matrix3d BuildInertiaTensor( double Ixx, double Iyy, double Izz,
                                           double Ixy, double Ixz, double Iyz)
{
  Eigen::Matrix3d I;
  I <<  Ixx, -Ixy, -Ixz,
       -Ixy,  Iyy, -Iyz,
       -Ixz, -Iyz,  Izz;
  return I;
}

CentroidalModel::Jac
BuildCrossProductMatrix(const Eigen::Vector3d& in)
{
  CentroidalModel::Jac out(3,3);

  out.coeffRef(0,1) = -in(2); out.coeffRef(0,2) =  in(1);
  out.coeffRef(1,0) =  in(2); out.coeffRef(1,2) = -in(0);
  out.coeffRef(2,0) = -in(1); out.coeffRef(2,1) =  in(0);

  return out;
}

CentroidalModel::CentroidalModel (double mass,
                                  double Ixx, double Iyy, double Izz,
                                  double Ixy, double Ixz, double Iyz,
                                  int ee_count)
   : CentroidalModel(mass,
                     BuildInertiaTensor(Ixx, Iyy, Izz, Ixy, Ixz, Iyz),
                     ee_count)
{
}

CentroidalModel::CentroidalModel (double mass, const Eigen::Matrix3d& inertia,
                                  int ee_count)
    :DynamicModel(mass)
{
  SetCurrent(ComPos::Zero(), AngVel::Zero(), EELoad(ee_count), EEPos(ee_count));
  I_dense_ = inertia;
  I_       = inertia.sparseView();
  I_inv_   = inertia.inverse().sparseView();
}

CentroidalModel::BaseAcc
CentroidalModel::GetBaseAccelerationInWorld () const
{
  Vector3d f_lin, f_ang; f_lin.setZero(); f_ang.setZero();

  for (int ee=0; ee<ee_pos_.size(); ++ee) {
    Vector3d f = ee_force_.at(ee);
    f_ang += f.cross(com_pos_-ee_pos_.at(ee));
    f_lin += f;
  }

  // moved gravity to bounds, as this is constant and would mess up SNOPT
  // static const Vector3d fg_W(0.0, 0.0, -m_*kGravity);
  // f_lin += fg_W;

  BaseAcc acc;
  acc.segment(AX, k3D) = I_inv_*(f_ang - omega_.cross(I_dense_*omega_)); // coriolis terms
  acc.segment(LX, k3D) = 1./m() *f_lin;

  return acc;
}

CentroidalModel::Jac
CentroidalModel::GetJacobianOfAccWrtBaseLin (const Jac& jac_pos_base_lin) const
{
  // build the com jacobian
  int n = jac_pos_base_lin.cols();

  Jac jac_ang(k3D, n);
  for (const Vector3d& f : ee_force_) {
    Jac jac_comp = BuildCrossProductMatrix(f)*jac_pos_base_lin;
    jac_ang += jac_comp;
  }

  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = I_inv_*jac_ang;

  // linear acceleration does not depend on base
  return jac;
}

CentroidalModel::Jac
CentroidalModel::GetJacobianOfAccWrtBaseAng (const Jac& jac_ang_vel) const
{
  int n = jac_ang_vel.cols();

  // the 6D base acceleration does not depend on base orientation, but on angular velocity
  // add derivative of w x Iw here!!!
  Jac jac_coriolis  = -BuildCrossProductMatrix(I_*omega_)*jac_ang_vel;
  jac_coriolis          +=  BuildCrossProductMatrix(omega_)*I_*jac_ang_vel;

  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = I_inv_*(-jac_coriolis);

  return jac;
}

CentroidalModel::Jac
CentroidalModel::GetJacobianofAccWrtForce (const Jac& ee_force_jac,
                                           EE ee) const
{
  Vector3d r = com_pos_-ee_pos_.at(ee);
  Jac jac_ang = -BuildCrossProductMatrix(r)*ee_force_jac;

  int n = ee_force_jac.cols();
  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = I_inv_*jac_ang;
  jac.middleRows(LX, k3D) = 1./m()*ee_force_jac;

  return jac;
}

CentroidalModel::Jac
CentroidalModel::GetJacobianofAccWrtEEPos (const Jac& jac_ee_pos,
                                           EE ee) const
{
  Vector3d f = ee_force_.at(ee);
  Jac jac_ang = BuildCrossProductMatrix(f)*(-jac_ee_pos);

  Jac jac(k6D, jac_ang.cols());
  jac.middleRows(AX, k3D) = I_inv_*jac_ang;
  // linear acceleration does not depend on endeffector position.
  return jac;
}

} /* namespace towr */
