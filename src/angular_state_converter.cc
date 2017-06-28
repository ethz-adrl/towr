/**
 @file    angular_state_converter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 27, 2017
 @brief   Brief description
 */

#include <xpp/opt/angular_state_converter.h>

#include <kindr/Core>

namespace xpp {
namespace opt {

// Euler angles stored in roll, pitch, yaw order in vector
// still using ZYX convention when applying them though.
enum EulerCoords {EX=0, EY=1, EZ=2};

AngularStateConverter::AngularStateConverter ()
{
  // TODO Auto-generated constructor stub
}

AngularStateConverter::AngularStateConverter (const OrientationVariables& euler)
{
  euler_ = euler;
}

AngularStateConverter::~AngularStateConverter ()
{
  // TODO Auto-generated destructor stub
}

AngularStateConverter::AngularVel
AngularStateConverter::GetAngularVelocity (double t) const
{
  StateLin3d ori = euler_->GetPoint(t);
  AngularVel w = GetM(ori.p_)*ori.v_;
  return w;
}

AngularStateConverter::AngularAcc
AngularStateConverter::GetAngularAcceleration (double t) const
{
  StateLin3d ori = euler_->GetPoint(t);
  AngularAcc wd = GetMdot(ori.p_, ori.v_)*ori.v_ + GetM(ori.p_)*ori.a_;
  return wd;
}

Jacobian
AngularStateConverter::GetDerivOfAngAccWrtCoeff (double t) const
{

  int n_coeff = euler_->GetRows();
  Jacobian jac(kDim3d, n_coeff);

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

    jac.row(dim) =   vel * dMdot_du
                   + GetMdot(ori.p_, ori.v_).row(dim)*dVel_du
                   + acc * dM_du
                   + GetM(ori.p_).row(dim) * dAcc_du;
  }

  return jac;
}

MatrixSXd
AngularStateConverter::GetM (const EulerAngles& xyz) const
{
  double z = xyz(EZ);
  double y = xyz(EY);
  Jacobian M(kDim3d, kDim3d);

                          M.coeffRef(0,EY) = -sin(z);  M.coeffRef(0,EX) =  cos(y)*cos(z);
                          M.coeffRef(1,EY) =  cos(z);  M.coeffRef(1,EX) =  cos(y)*sin(z);
  M.coeffRef(2,EZ) = 1.0;                              M.coeffRef(2,EX) =  -sin(y);

  return M;
}

MatrixSXd
AngularStateConverter::GetMdot (const EulerAngles& xyz,
                                const EulerRates& xyz_d) const
{
  double z  = xyz(EZ);
  double zd = xyz_d(EZ);
  double y  = xyz(EY);
  double yd = xyz_d(EY);

  Jacobian Mdot(kDim3d, kDim3d);

  Mdot.coeffRef(0,EY) = -cos(z)*zd; Mdot.coeffRef(0,EX) = -cos(z)*sin(y)*yd - cos(y)*sin(z)*zd;
  Mdot.coeffRef(1,EY) = -sin(z)*zd; Mdot.coeffRef(1,EX) =  cos(y)*cos(z)*zd - sin(y)*sin(z)*yd;
                                    Mdot.coeffRef(2,EX) = -cos(y)*yd;

 return Mdot;
}

Jacobian
AngularStateConverter::GetDerivMwrtCoeff (double t, Coords3D ang_acc_dim) const
{
  int n_coeff    = euler_->GetRows();
  StateLin3d ori = euler_->GetPoint(t);

  double z = ori.p_(EZ);
  double y = ori.p_(EY);
  JacobianRow jac_z = euler_->GetJacobian(t, kPos, EZ);
  JacobianRow jac_y = euler_->GetJacobian(t, kPos, EY);

  Jacobian jac(kDim3d,n_coeff);

  switch (ang_acc_dim) {
    case X: // basically derivative of top row (3 elements) of matrix M
      jac.row(EY) = -cos(z)*jac_z;
      jac.row(EX) = -cos(z)*sin(y)*jac_y - cos(y)*sin(z)*jac_z;
      break;
    case Y: // middle row of M
      jac.row(EY) = -sin(z)*jac_z;
      jac.row(EX) = cos(y)*cos(z)*jac_z - sin(y)*sin(z)*jac_y;
      break;
    case Z: // bottom row of M
      jac.row(EX) = -cos(y)*jac_y;
      break;
    default:
      assert(false);
      break;
  }

  return jac;
}

MatrixSXd
AngularStateConverter::GetRotationMatrixWorldToBase (double t) const
{
  StateLin3d ori = euler_->GetPoint(t);
  return GetRotationMatrixWorldToBase(ori.p_);
}

MatrixSXd
AngularStateConverter::GetRotationMatrixWorldToBase (const EulerAngles& xyz)
{

  double x = xyz(EX);
  double y = xyz(EY);
  double z = xyz(EZ);

  Eigen::Matrix3d M;
  M << cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
      cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x),
            -sin(y),                        cos(y)*sin(x),                        cos(x)*cos(y);

  return M.sparseView(1.0, -1.0);
}

Jacobian
AngularStateConverter::GetDerivativeOfRotationMatrixRowWrtCoeff (double t,
                                                                 Coords3D row) const
{
  StateLin3d ori = euler_->GetPoint(t);
  double x = ori.p_(EX);
  double y = ori.p_(EY);
  double z = ori.p_(EZ);

  JacobianRow jac_x = euler_->GetJacobian(t, kPos, EX);
  JacobianRow jac_y = euler_->GetJacobian(t, kPos, EY);
  JacobianRow jac_z = euler_->GetJacobian(t, kPos, EZ);

  int n_coeff = euler_->GetRows();
  Jacobian jac(kDim3d,n_coeff);

  switch (row) {
    case X: // basically derivative of top row (3 elements) of matrix R
      jac.row(EX) = -cos(z)*sin(y)*jac_y - cos(y)*sin(z)*jac_z;
      jac.row(EY) = sin(x)*sin(z)*jac_x - cos(x)*cos(z)*jac_z - sin(x)*sin(y)*sin(z)*jac_z + cos(x)*cos(z)*sin(y)*jac_x + cos(y)*cos(z)*sin(x)*jac_y;
      jac.row(EZ) = cos(x)*sin(z)*jac_x + cos(z)*sin(x)*jac_z - cos(z)*sin(x)*sin(y)*jac_x - cos(x)*sin(y)*sin(z)*jac_z + cos(x)*cos(y)*cos(z)*jac_y;
      break;
    case Y: // middle row of R
      jac.row(EX) = cos(y)*cos(z)*jac_z - sin(y)*sin(z)*jac_y;
      jac.row(EY) = cos(x)*sin(y)*sin(z)*jac_x - cos(x)*sin(z)*jac_z - cos(z)*sin(x)*jac_x + cos(y)*sin(x)*sin(z)*jac_y + cos(z)*sin(x)*sin(y)*jac_z;
      jac.row(EZ) = sin(x)*sin(z)*jac_z - cos(x)*cos(z)*jac_x - sin(x)*sin(y)*sin(z)*jac_x + cos(x)*cos(y)*sin(z)*jac_y + cos(x)*cos(z)*sin(y)*jac_z;
      break;
    case Z: // bottom row of R
      jac.row(EX) = -cos(y)*jac_y;
      jac.row(EY) =  cos(x)*cos(y)*jac_x - sin(x)*sin(y)*jac_y;
      jac.row(EZ) = - cos(y)*sin(x)*jac_x - cos(x)*sin(y)*jac_y;
      break;
    default:
      assert(false);
      break;
  }

  return jac;
}

Jacobian
AngularStateConverter::GetDerivMdotwrtCoeff (double t, Coords3D ang_acc_dim) const
{
  int n_coeff    = euler_->GetRows();
  StateLin3d ori = euler_->GetPoint(t);

  double z  = ori.p_(EZ);
  double zd = ori.v_(EZ);
  double y  = ori.p_(EY);
  double yd = ori.v_(EY);

  JacobianRow jac_z  = euler_->GetJacobian(t, kPos, EZ);
  JacobianRow jac_y  = euler_->GetJacobian(t, kPos, EY);
  JacobianRow jac_zd = euler_->GetJacobian(t, kVel, EZ);
  JacobianRow jac_yd = euler_->GetJacobian(t, kVel, EY);

  Jacobian jac(kDim3d,n_coeff);
  switch (ang_acc_dim) {
    case X: // derivative of top row (3 elements) of matrix M-dot
      jac.row(EY) = sin(z)*zd*jac_z - cos(z)*jac_zd;
      jac.row(EX) = sin(y)*sin(z)*yd*jac_z - cos(y)*sin(z)*jac_zd - cos(y)*cos(z)*yd*jac_y - cos(y)*cos(z)*zd*jac_z - cos(z)*sin(y)*jac_yd + sin(y)*sin(z)*jac_y*zd;
      break;
    case Y: // middle row of M
      jac.row(EY) = - sin(z)*jac_zd - cos(z)*zd*jac_z;
      jac.row(EX) = cos(y)*cos(z)*jac_zd - sin(y)*sin(z)*jac_yd - cos(y)*sin(z)*yd*jac_y - cos(z)*sin(y)*yd*jac_z - cos(z)*sin(y)*jac_y*zd - cos(y)*sin(z)*zd*jac_z;
      break;
    case Z: // bottom Row of M
      jac.row(EX) = sin(y)*yd*jac_y - cos(y)*jac_yd;
      break;
    default:
      assert(false);
      break;
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */























