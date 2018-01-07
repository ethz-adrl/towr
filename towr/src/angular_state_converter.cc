/**
 @file    angular_state_converter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 27, 2017
 @brief   Brief description
 */

#include <towr/variables/angular_state_converter.h>

#include <cassert>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace towr {

using namespace xpp;

AngularStateConverter::AngularStateConverter (const Spline::Ptr& euler)
{
  euler_ = euler;
}

StateAng3d
AngularStateConverter::GetState (const StateLin3d& euler)
{
  StateAng3d ang;

  ang.q = AngularStateConverter::GetOrientation(euler.p_);
  ang.w = AngularStateConverter::GetAngularVelocity(euler.p_, euler.v_);
  ang.wd = AngularStateConverter::GetAngularAcceleration(euler);

  return ang;
}

Eigen::Quaterniond
AngularStateConverter::GetOrientation (const EulerAngles& pos)
{
  Eigen::Matrix3d R_WB = GetRotationMatrixBaseToWorld(pos);
  return Eigen::Quaterniond(R_WB);
}

AngularStateConverter::AngularVel
AngularStateConverter::GetAngularVelocity (double t) const
{
  StateLin3d ori = euler_->GetPoint(t);
  return GetAngularVelocity(ori.p_, ori.v_);
}

AngularStateConverter::AngularVel
AngularStateConverter::GetAngularVelocity (const EulerAngles& pos,
                                           const EulerAngles& vel)
{
  return GetM(pos)*vel;
}

AngularStateConverter::Jacobian
AngularStateConverter::GetDerivOfAngVelWrtCoeff(double t) const
{
  Jacobian jac(kDim3d, OptVariablesOfCurrentPolyCount(t));

  StateLin3d ori = euler_->GetPoint(t);
  // convert to sparse, but also regard 0.0 as non-zero element, because
  // could turn nonzero during the course of the program
  JacobianRow vel = ori.v_.transpose().sparseView(1.0, -1.0);
  Jacobian dVel_du  = euler_->GetJacobian(t, kVel);

  for (auto dim : {X,Y,Z}) {
    Jacobian dM_du = GetDerivMwrtCoeff(t,dim);
    jac.row(dim) = vel*dM_du + GetM(ori.p_).row(dim)*dVel_du;
  }

  return jac;
}

AngularStateConverter::AngularAcc
AngularStateConverter::GetAngularAcceleration (double t) const
{
  StateLin3d ori = euler_->GetPoint(t);
  return GetAngularAcceleration(ori);
}

AngularStateConverter::AngularAcc
AngularStateConverter::GetAngularAcceleration (StateLin3d ori)
{
  return GetMdot(ori.p_, ori.v_)*ori.v_ + GetM(ori.p_)*ori.a_;
}

AngularStateConverter::Jacobian
AngularStateConverter::GetDerivOfAngAccWrtCoeff (double t) const
{
  Jacobian jac(kDim3d, OptVariablesOfCurrentPolyCount(t));


  StateLin3d ori = euler_->GetPoint(t);
  // convert to sparse, but also regard 0.0 as non-zero element, because
  // could turn nonzero during the course of the program
  JacobianRow vel = ori.v_.transpose().sparseView(1.0, -1.0);
  JacobianRow acc = ori.a_.transpose().sparseView(1.0, -1.0);

  Jacobian dVel_du  = euler_->GetJacobian(t, kVel);
  Jacobian dAcc_du  = euler_->GetJacobian(t, kAcc);


  for (auto dim : {X,Y,Z}) {

    Jacobian dMdot_du = GetDerivMdotwrtCoeff(t,dim);
    Jacobian dM_du    = GetDerivMwrtCoeff(t,dim);

    jac.row(dim) = vel                               * dMdot_du
                   + GetMdot(ori.p_, ori.v_).row(dim)* dVel_du
                   + acc                             * dM_du
                   + GetM(ori.p_).row(dim)           * dAcc_du;
  }

  return jac;
}

AngularStateConverter::MatrixSXd
AngularStateConverter::GetM (const EulerAngles& xyz)
{
  double z = xyz(Z);
  double y = xyz(Y);

  // Euler ZYX rates to angular velocity
  // http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
  Jacobian M(kDim3d, kDim3d);

                          M.coeffRef(0,Y) = -sin(z);  M.coeffRef(0,X) =  cos(y)*cos(z);
                          M.coeffRef(1,Y) =  cos(z);  M.coeffRef(1,X) =  cos(y)*sin(z);
  M.coeffRef(2,Z) = 1.0;                              M.coeffRef(2,X) =  -sin(y);

  return M;
}

AngularStateConverter::MatrixSXd
AngularStateConverter::GetMdot (const EulerAngles& xyz,
                                const EulerRates& xyz_d)
{
  double z  = xyz(Z);
  double zd = xyz_d(Z);
  double y  = xyz(Y);
  double yd = xyz_d(Y);

  Jacobian Mdot(kDim3d, kDim3d);

  Mdot.coeffRef(0,Y) = -cos(z)*zd; Mdot.coeffRef(0,X) = -cos(z)*sin(y)*yd - cos(y)*sin(z)*zd;
  Mdot.coeffRef(1,Y) = -sin(z)*zd; Mdot.coeffRef(1,X) =  cos(y)*cos(z)*zd - sin(y)*sin(z)*yd;
                                   Mdot.coeffRef(2,X) = -cos(y)*yd;

 return Mdot;
}

AngularStateConverter::Jacobian
AngularStateConverter::GetDerivMwrtCoeff (double t, Coords3D ang_acc_dim) const
{
  StateLin3d ori = euler_->GetPoint(t);

  double z = ori.p_(Z);
  double y = ori.p_(Y);
  JacobianRow jac_z = GetJac(t, kPos, Z);
  JacobianRow jac_y = GetJac(t, kPos, Y);

  Jacobian jac(kDim3d, OptVariablesOfCurrentPolyCount(t));

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

AngularStateConverter::MatrixSXd
AngularStateConverter::GetRotationMatrixBaseToWorld (double t) const
{
  StateLin3d ori = euler_->GetPoint(t);
  return GetRotationMatrixBaseToWorld(ori.p_);
}

AngularStateConverter::MatrixSXd
AngularStateConverter::GetRotationMatrixBaseToWorld (const EulerAngles& xyz)
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

AngularStateConverter::Jacobian
AngularStateConverter::GetDerivativeOfRotationMatrixRowWrtCoeff (double t,
                                                                 const Vector3d& v,
                                                                 bool inverse) const
{
  JacRowMatrix Rd = GetDerivativeOfRotationMatrixWrtCoeff(t);
  Jacobian jac(kDim3d, OptVariablesOfCurrentPolyCount(t));

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

AngularStateConverter::JacRowMatrix
AngularStateConverter::GetDerivativeOfRotationMatrixWrtCoeff (double t) const
{
  JacRowMatrix jac;

  StateLin3d ori = euler_->GetPoint(t);
  double x = ori.p_(X);
  double y = ori.p_(Y);
  double z = ori.p_(Z);

  JacobianRow jac_x = GetJac(t, kPos, X);
  JacobianRow jac_y = GetJac(t, kPos, Y);
  JacobianRow jac_z = GetJac(t, kPos, Z);

  jac.at(X).at(X) = -cos(z)*sin(y)*jac_y - cos(y)*sin(z)*jac_z;
  jac.at(X).at(Y) = sin(x)*sin(z)*jac_x - cos(x)*cos(z)*jac_z - sin(x)*sin(y)*sin(z)*jac_z + cos(x)*cos(z)*sin(y)*jac_x + cos(y)*cos(z)*sin(x)*jac_y;
  jac.at(X).at(Z) = cos(x)*sin(z)*jac_x + cos(z)*sin(x)*jac_z - cos(z)*sin(x)*sin(y)*jac_x - cos(x)*sin(y)*sin(z)*jac_z + cos(x)*cos(y)*cos(z)*jac_y;

  jac.at(Y).at(X) = cos(y)*cos(z)*jac_z - sin(y)*sin(z)*jac_y;
  jac.at(Y).at(Y) = cos(x)*sin(y)*sin(z)*jac_x - cos(x)*sin(z)*jac_z - cos(z)*sin(x)*jac_x + cos(y)*sin(x)*sin(z)*jac_y + cos(z)*sin(x)*sin(y)*jac_z;
  jac.at(Y).at(Z) = sin(x)*sin(z)*jac_z - cos(x)*cos(z)*jac_x - sin(x)*sin(y)*sin(z)*jac_x + cos(x)*cos(y)*sin(z)*jac_y + cos(x)*cos(z)*sin(y)*jac_z;

  jac.at(Z).at(X) = -cos(y)*jac_y;
  jac.at(Z).at(Y) =  cos(x)*cos(y)*jac_x - sin(x)*sin(y)*jac_y;
  jac.at(Z).at(Z) = - cos(y)*sin(x)*jac_x - cos(x)*sin(y)*jac_y;

  return jac;
}

AngularStateConverter::Jacobian
AngularStateConverter::GetDerivMdotwrtCoeff (double t, Coords3D ang_acc_dim) const
{
  StateLin3d ori = euler_->GetPoint(t);

  double z  = ori.p_(Z);
  double zd = ori.v_(Z);
  double y  = ori.p_(Y);
  double yd = ori.v_(Y);

  JacobianRow jac_z  = GetJac(t, kPos, Z);
  JacobianRow jac_y  = GetJac(t, kPos, Y);
  JacobianRow jac_zd = GetJac(t, kVel, Z);
  JacobianRow jac_yd = GetJac(t, kVel, Y);

  Jacobian jac(kDim3d, OptVariablesOfCurrentPolyCount(t));
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

AngularStateConverter::JacobianRow
AngularStateConverter::GetJac (double t, MotionDerivative deriv, Coords3D dim) const
{
  return euler_->GetJacobian(t, deriv).row(dim);
}

int
AngularStateConverter::OptVariablesOfCurrentPolyCount (double t) const
{
  // zmp_ attention, not the same thing
  return euler_->GetJacobian(t, kPos).cols();
}

} /* namespace towr */

