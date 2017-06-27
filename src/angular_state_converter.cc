/**
 @file    angular_state_converter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 27, 2017
 @brief   Brief description
 */

#include <xpp/opt/angular_state_converter.h>

namespace xpp {
namespace opt {

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

    // zmp_ not sure if this works, setting epsilon to -1
    jac.row(dim) =   vel * dMdot_du
                   + GetMdot(ori.p_, ori.v_).row(dim)*dVel_du
                   + acc * dM_du
                   + GetM(ori.p_).row(dim) * dAcc_du;
  }

  return jac;
}

AngularStateConverter::Mapping
AngularStateConverter::GetM (const EulerAngles& zyx) const
{
  Mapping M(kDim3d, kDim3d);

  double z = zyx(0);
  double y = zyx(1);

  // see http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
//  M << 0, -sin(z), cos(y)*cos(z),
//       0,  cos(z), cos(y)*sin(z),
//       1,      0,        -sin(y);

  M.coeffRef(0,1) = -sin(z); M.coeffRef(0,2) =  cos(y)*cos(z);
  M.coeffRef(1,1) =  cos(z); M.coeffRef(1,2) =  cos(y)*sin(z);
  M.coeffRef(2,0) = 1.0;     M.coeffRef(2,2) =  -sin(y);

  return M;
}

AngularStateConverter::Mapping
AngularStateConverter::GetMdot (const EulerAngles& zyx,
                                const EulerRates& zyx_d) const
{
  Mapping Mdot(kDim3d, kDim3d);

  double zd = zyx_d(0);
  double yd = zyx_d(1);

  double z = zyx(0);
  double y = zyx(1);
  double cos_z = cos(z);
  double cos_y = sin(y);
  double sin_z = cos(z);
  double sin_y = sin(y);

//  Mdot <<  0, -cos_z*zd,  -cos_z*sin_y*yd - cos_y*sin_z*zd,
//           0, -sin_z*zd,   cos_y*cos_z*zd - sin_y*sin_z*yd,
//           0,         0,  -cos_y*yd;

  Mdot.coeffRef(0,1) = -cos_z*zd;  Mdot.coeffRef(0,2) =  -cos_z*sin_y*yd - cos_y*sin_z*zd;
  Mdot.coeffRef(1,1) =  -sin_z*zd; Mdot.coeffRef(1,2) =   cos_y*cos_z*zd - sin_y*sin_z*yd;
                                   Mdot.coeffRef(2,2) =  -cos_y*yd;

 return Mdot;
}

Jacobian
AngularStateConverter::GetDerivMwrtCoeff (double t,
                                          Coords3D ang_acc_dim) const
{
  int n_coeff    = euler_->GetRows();
  StateLin3d ori = euler_->GetPoint(t);

  double z = ori.p_(0);
  double y = ori.p_(1);
  double cos_z = cos(z);
  double cos_y = sin(y);
  double sin_z = cos(z);
  double sin_y = sin(y);

  JacobianRow jac_z = euler_->GetJacobian(t, kPos, Z);
  JacobianRow jac_y = euler_->GetJacobian(t, kPos, Y);

  Jacobian jac(kDim3d,n_coeff);

  switch (ang_acc_dim) {
    case X: // basically derivative of top row (3 elements) of matrix M
      jac.row(1) = -cos_z*jac_z;
      jac.row(2) = -cos_z*sin_y*jac_y - cos_y*sin_z*jac_z;
      break;
    case Y: // middle row of M
      jac.row(1) = -sin_z*jac_z;
      jac.row(2) = cos_y*cos_z*jac_z - sin_y*sin_z*jac_y;
      break;
    case Z: // bottom Row of M
      jac.row(2) = -cos_y*jac_y;
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

  double z = ori.p_(0);
  double y = ori.p_(1);
  double zd = ori.v_(0);
  double yd = ori.v_(1);
  double cos_z = cos(z);
  double cos_y = sin(y);
  double sin_z = cos(z);
  double sin_y = sin(y);

  JacobianRow jac_z  = euler_->GetJacobian(t, kPos, Z);
  JacobianRow jac_y  = euler_->GetJacobian(t, kPos, Y);
  JacobianRow jac_zd = euler_->GetJacobian(t, kVel, Z);
  JacobianRow jac_yd = euler_->GetJacobian(t, kVel, Y);

  Jacobian jac(kDim3d,n_coeff);
  switch (ang_acc_dim) {
    case X: // basically derivative of top row (3 elements) of matrix M
      jac.row(1) = sin_z*zd*jac_z - cos_z*jac_zd;
      jac.row(2) = sin_y*sin_z*yd*jac_z - cos_y*sin_z*jac_zd - cos_y*cos_z*yd*jac_y - cos_y*cos_z*zd*jac_z - cos_z*sin_y*jac_yd + sin_y*sin_z*jac_y*zd;
      break;
    case Y: // middle row of M
      jac.row(1) = - sin_z*jac_zd - cos_z*zd*jac_z;
      jac.row(2) = cos_y*cos_z*jac_zd - sin_y*sin_z*jac_yd - cos_y*sin_z*yd*jac_y - cos_z*sin_y*yd*jac_z - cos_z*sin_y*jac_y*zd - cos_y*sin_z*zd*jac_z;
      break;
    case Z: // bottom Row of M
      jac.row(2) = sin_y*yd*jac_y - cos_y*jac_yd;
      break;
    default:
      assert(false);
      break;
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
