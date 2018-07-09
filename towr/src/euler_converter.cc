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

#include <towr/variables/euler_converter.h>

#include <cassert>
#include <cmath>

namespace towr {


EulerConverter::EulerConverter (const NodeSpline::Ptr& euler)
{
  euler_ = euler;
  jac_wrt_nodes_structure_ = Jacobian(k3D, euler->GetNodeVariablesCount());
}

Eigen::Quaterniond
EulerConverter::GetQuaternionBaseToWorld (double t) const
{
  State ori = euler_->GetPoint(t);
  return GetQuaternionBaseToWorld(ori.p());
}

Eigen::Quaterniond
EulerConverter::GetQuaternionBaseToWorld (const EulerAngles& pos)
{
  Eigen::Matrix3d R_WB = GetRotationMatrixBaseToWorld(pos);
  return Eigen::Quaterniond(R_WB);
}

Eigen::Vector3d
EulerConverter::GetAngularVelocityInWorld (double t) const
{
  State ori = euler_->GetPoint(t);
  return GetAngularVelocityInWorld(ori.p(), ori.v());
}

Eigen::Vector3d
EulerConverter::GetAngularVelocityInWorld (const EulerAngles& pos,
                                           const EulerRates& vel)
{
  return GetM(pos)*vel;
}

Eigen::Vector3d
EulerConverter::GetAngularAccelerationInWorld (double t) const
{
  State ori = euler_->GetPoint(t);
  return GetAngularAccelerationInWorld(ori);
}

Eigen::Vector3d
EulerConverter::GetAngularAccelerationInWorld (State ori)
{
  return GetMdot(ori.p(), ori.v())*ori.v() + GetM(ori.p())*ori.a();
}

EulerConverter::Jacobian
EulerConverter::GetDerivOfAngVelWrtEulerNodes(double t) const
{
  Jacobian jac = jac_wrt_nodes_structure_;

  State ori = euler_->GetPoint(t);
  // convert to sparse, but also regard 0.0 as non-zero element, because
  // could turn nonzero during the course of the program
  JacobianRow vel = ori.v().transpose().sparseView(1.0, -1.0);
  Jacobian dVel_du  = euler_->GetJacobianWrtNodes(t, kVel);

  for (auto dim : {X,Y,Z}) {
    Jacobian dM_du = GetDerivMwrtNodes(t,dim);
    jac.row(dim) = vel*dM_du + GetM(ori.p()).row(dim)*dVel_du;
  }

  return jac;
}

EulerConverter::Jacobian
EulerConverter::GetDerivOfAngAccWrtEulerNodes (double t) const
{
  Jacobian jac = jac_wrt_nodes_structure_;


  State ori = euler_->GetPoint(t);
  // convert to sparse, but also regard 0.0 as non-zero element, because
  // could turn nonzero during the course of the program
  JacobianRow vel = ori.v().transpose().sparseView(1.0, -1.0);
  JacobianRow acc = ori.a().transpose().sparseView(1.0, -1.0);

  Jacobian dVel_du  = euler_->GetJacobianWrtNodes(t, kVel);
  Jacobian dAcc_du  = euler_->GetJacobianWrtNodes(t, kAcc);


  for (auto dim : {X,Y,Z}) {
    Jacobian dMdot_du = GetDerivMdotwrtNodes(t,dim);
    Jacobian dM_du    = GetDerivMwrtNodes(t,dim);

    jac.row(dim) = vel                               * dMdot_du
                   + GetMdot(ori.p(), ori.v()).row(dim)* dVel_du
                   + acc                             * dM_du
                   + GetM(ori.p()).row(dim)           * dAcc_du;
  }

  return jac;
}

EulerConverter::MatrixSXd
EulerConverter::GetM (const EulerAngles& xyz)
{
  double z = xyz(Z);
  double y = xyz(Y);

  // Euler ZYX rates to angular velocity
  // http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
  Jacobian M(k3D, k3D);

       /* - */           M.coeffRef(0,Y) = -sin(z);  M.coeffRef(0,X) =  cos(y)*cos(z);
       /* - */           M.coeffRef(1,Y) =  cos(z);  M.coeffRef(1,X) =  cos(y)*sin(z);
  M.coeffRef(2,Z) = 1.0;          /* - */            M.coeffRef(2,X) =  -sin(y);

  return M;
}

EulerConverter::MatrixSXd
EulerConverter::GetMdot (const EulerAngles& xyz,
                         const EulerRates& xyz_d)
{
  double z  = xyz(Z);
  double zd = xyz_d(Z);
  double y  = xyz(Y);
  double yd = xyz_d(Y);

  Jacobian Mdot(k3D, k3D);

  Mdot.coeffRef(0,Y) = -cos(z)*zd; Mdot.coeffRef(0,X) = -cos(z)*sin(y)*yd - cos(y)*sin(z)*zd;
  Mdot.coeffRef(1,Y) = -sin(z)*zd; Mdot.coeffRef(1,X) =  cos(y)*cos(z)*zd - sin(y)*sin(z)*yd;
              /* - */              Mdot.coeffRef(2,X) = -cos(y)*yd;

 return Mdot;
}

EulerConverter::Jacobian
EulerConverter::GetDerivMwrtNodes (double t, Dim3D ang_acc_dim) const
{
  State ori = euler_->GetPoint(t);

  double z = ori.p()(Z);
  double y = ori.p()(Y);
  JacobianRow jac_z = GetJac(t, kPos, Z);
  JacobianRow jac_y = GetJac(t, kPos, Y);

  Jacobian jac = jac_wrt_nodes_structure_;

  switch (ang_acc_dim) {
    case X: // basically derivative of top row (3 elements) of matrix M
      jac.row(Y) = -cos(z)*jac_z;
      jac.row(X) = -cos(z)*sin(y)*jac_y - cos(y)*sin(z)*jac_z;
      break;
    case Y: // middle row of M
      jac.row(Y) = -sin(z)*jac_z;
      jac.row(X) = cos(y)*cos(z)*jac_z - sin(y)*sin(z)*jac_y;
      break;
    case Z: // bottom row of M
      jac.row(X) = -cos(y)*jac_y;
      break;
    default:
      assert(false);
      break;
  }

  return jac;
}

EulerConverter::MatrixSXd
EulerConverter::GetRotationMatrixBaseToWorld (double t) const
{
  State ori = euler_->GetPoint(t);
  return GetRotationMatrixBaseToWorld(ori.p());
}

EulerConverter::MatrixSXd
EulerConverter::GetRotationMatrixBaseToWorld (const EulerAngles& xyz)
{
  double x = xyz(X);
  double y = xyz(Y);
  double z = xyz(Z);

  Eigen::Matrix3d M;
  //  http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf (Euler ZYX)
  M << cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
       cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x),
             -sin(y),                        cos(y)*sin(x),                        cos(x)*cos(y);

  return M.sparseView(1.0, -1.0);
}

EulerConverter::Jacobian
EulerConverter::DerivOfRotVecMult (double t, const Vector3d& v, bool inverse) const
{
  JacRowMatrix Rd = GetDerivativeOfRotationMatrixWrtNodes(t);
  Jacobian jac = jac_wrt_nodes_structure_;

  for (int row : {X,Y,Z}) {
    for (int col : {X, Y, Z}) {
      // since for every rotation matrix R^(-1) = R^T, just swap rows and
      // columns for calculation of derivative of inverse rotation matrix
      JacobianRow jac_row = inverse? Rd.at(col).at(row) : Rd.at(row).at(col);
      jac.row(row) += v(col)*jac_row;
    }
  }

  return jac;
}

EulerConverter::JacRowMatrix
EulerConverter::GetDerivativeOfRotationMatrixWrtNodes (double t) const
{
  JacRowMatrix jac;

  State ori = euler_->GetPoint(t);
  double x = ori.p()(X);
  double y = ori.p()(Y);
  double z = ori.p()(Z);

  JacobianRow jac_x = GetJac(t, kPos, X);
  JacobianRow jac_y = GetJac(t, kPos, Y);
  JacobianRow jac_z = GetJac(t, kPos, Z);

  jac.at(X).at(X) = -cos(z)*sin(y)*jac_y - cos(y)*sin(z)*jac_z;
  jac.at(X).at(Y) =  sin(x)*sin(z)*jac_x - cos(x)*cos(z)*jac_z - sin(x)*sin(y)*sin(z)*jac_z + cos(x)*cos(z)*sin(y)*jac_x + cos(y)*cos(z)*sin(x)*jac_y;
  jac.at(X).at(Z) =  cos(x)*sin(z)*jac_x + cos(z)*sin(x)*jac_z - cos(z)*sin(x)*sin(y)*jac_x - cos(x)*sin(y)*sin(z)*jac_z + cos(x)*cos(y)*cos(z)*jac_y;

  jac.at(Y).at(X) = cos(y)*cos(z)*jac_z - sin(y)*sin(z)*jac_y;
  jac.at(Y).at(Y) = cos(x)*sin(y)*sin(z)*jac_x - cos(x)*sin(z)*jac_z - cos(z)*sin(x)*jac_x + cos(y)*sin(x)*sin(z)*jac_y + cos(z)*sin(x)*sin(y)*jac_z;
  jac.at(Y).at(Z) = sin(x)*sin(z)*jac_z - cos(x)*cos(z)*jac_x - sin(x)*sin(y)*sin(z)*jac_x + cos(x)*cos(y)*sin(z)*jac_y + cos(x)*cos(z)*sin(y)*jac_z;

  jac.at(Z).at(X) = -cos(y)*jac_y;
  jac.at(Z).at(Y) =  cos(x)*cos(y)*jac_x - sin(x)*sin(y)*jac_y;
  jac.at(Z).at(Z) = -cos(y)*sin(x)*jac_x - cos(x)*sin(y)*jac_y;

  return jac;
}

EulerConverter::Jacobian
EulerConverter::GetDerivMdotwrtNodes (double t, Dim3D ang_acc_dim) const
{
  State ori = euler_->GetPoint(t);

  double z  = ori.p()(Z);
  double zd = ori.v()(Z);
  double y  = ori.p()(Y);
  double yd = ori.v()(Y);

  JacobianRow jac_z  = GetJac(t, kPos, Z);
  JacobianRow jac_y  = GetJac(t, kPos, Y);
  JacobianRow jac_zd = GetJac(t, kVel, Z);
  JacobianRow jac_yd = GetJac(t, kVel, Y);

  Jacobian jac = jac_wrt_nodes_structure_;
  switch (ang_acc_dim) {
    case X: // derivative of top row (3 elements) of matrix M-dot
      jac.row(Y) = sin(z)*zd*jac_z - cos(z)*jac_zd;
      jac.row(X) = sin(y)*sin(z)*yd*jac_z - cos(y)*sin(z)*jac_zd - cos(y)*cos(z)*yd*jac_y - cos(y)*cos(z)*zd*jac_z - cos(z)*sin(y)*jac_yd + sin(y)*sin(z)*jac_y*zd;
      break;
    case Y: // middle row of M
      jac.row(Y) = - sin(z)*jac_zd - cos(z)*zd*jac_z;
      jac.row(X) = cos(y)*cos(z)*jac_zd - sin(y)*sin(z)*jac_yd - cos(y)*sin(z)*yd*jac_y - cos(z)*sin(y)*yd*jac_z - cos(z)*sin(y)*jac_y*zd - cos(y)*sin(z)*zd*jac_z;
      break;
    case Z: // bottom Row of M
      jac.row(X) = sin(y)*yd*jac_y - cos(y)*jac_yd;
      break;
    default:
      assert(false);
      break;
  }

  return jac;
}

EulerConverter::JacobianRow
EulerConverter::GetJac (double t, Dx deriv, Dim3D dim) const
{
  return euler_->GetJacobianWrtNodes(t, deriv).row(dim);
}

} /* namespace towr */
