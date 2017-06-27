/**
 @file    angular_state_converter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 27, 2017
 @brief   Brief description
 */

#include <xpp/opt/angular_state_converter.h>

namespace xpp {
namespace opt {

// zmp_ fix this, hacky!
enum EulerCoords {EZ=0, EY=1, EX=2};

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

AngularStateConverter::Mapping
AngularStateConverter::GetM (const EulerAngles& zyx) const
{
  Mapping M(kDim3d, kDim3d);

  double z = zyx(EZ);
  double y = zyx(EY);

  // see http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
//  M << 0, -sin(z), cos(y)*cos(z),
//       0,  cos(z), cos(y)*sin(z),
//       1,      0,        -sin(y);

  M.coeffRef(0,1) = -sin(z); M.coeffRef(0,2) =  cos(y)*cos(z);
  M.coeffRef(1,1) =  cos(z); M.coeffRef(1,2) =  cos(y)*sin(z);
  M.coeffRef(2,0) =     1.0; M.coeffRef(2,2) =  -sin(y);

  return M;
}

AngularStateConverter::Mapping
AngularStateConverter::GetMdot (const EulerAngles& zyx,
                                const EulerRates& zyx_d) const
{

  double z = zyx(EZ);
  double y = zyx(EY);
  double zd = zyx_d(EZ);
  double yd = zyx_d(EY);

//  Mdot <<  0, -cos(z)*zd,  -cos(z)*sin(y)*yd - cos(y)*sin(z)*zd,
//           0, -sin(z)*zd,   cos(y)*cos(z)*zd - sin(y)*sin(z)*yd,
//           0,         0,   -cos(y)*yd;

  Mapping Mdot(kDim3d, kDim3d);

  Mdot.coeffRef(0,1) = -cos(z)*zd; Mdot.coeffRef(0,2) =  -cos(z)*sin(y)*yd - cos(y)*sin(z)*zd;
  Mdot.coeffRef(1,1) = -sin(z)*zd; Mdot.coeffRef(1,2) =   cos(y)*cos(z)*zd - sin(y)*sin(z)*yd;
                                   Mdot.coeffRef(2,2) =  -cos(y)*yd;

 return Mdot;
}

Jacobian
AngularStateConverter::GetDerivMwrtCoeff (double t,
                                          Coords3D ang_acc_dim) const
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
      jac.row(1) = -cos(z)*jac_z;
      jac.row(2) = -cos(z)*sin(y)*jac_y - cos(y)*sin(z)*jac_z;
      break;
    case Y: // middle row of M
      jac.row(1) = -sin(z)*jac_z;
      jac.row(2) = cos(y)*cos(z)*jac_z - sin(y)*sin(z)*jac_y;
      break;
    case Z: // bottom row of M
      jac.row(2) = -cos(y)*jac_y;
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
  double y  = ori.p_(EY);
  double zd = ori.v_(EZ);
  double yd = ori.v_(EY);

  // zmp_ these indices are definetely wrong!!! Z is now at first position
  JacobianRow jac_z  = euler_->GetJacobian(t, kPos, EZ);
  JacobianRow jac_y  = euler_->GetJacobian(t, kPos, EY);
  JacobianRow jac_zd = euler_->GetJacobian(t, kVel, EZ);
  JacobianRow jac_yd = euler_->GetJacobian(t, kVel, EY);

  Jacobian jac(kDim3d,n_coeff);
  switch (ang_acc_dim) {
    case X: // basically derivative of top row (3 elements) of matrix M
      jac.row(1) = sin(z)*zd*jac_z - cos(z)*jac_zd;
      jac.row(2) = sin(y)*sin(z)*yd*jac_z - cos(y)*sin(z)*jac_zd - cos(y)*cos(z)*yd*jac_y - cos(y)*cos(z)*zd*jac_z - cos(z)*sin(y)*jac_yd + sin(y)*sin(z)*jac_y*zd;
      break;
    case Y: // middle row of M
      jac.row(1) = - sin(z)*jac_zd - cos(z)*zd*jac_z;
      jac.row(2) = cos(y)*cos(z)*jac_zd - sin(y)*sin(z)*jac_yd - cos(y)*sin(z)*yd*jac_y - cos(z)*sin(y)*yd*jac_z - cos(z)*sin(y)*jac_y*zd - cos(y)*sin(z)*zd*jac_z;
      break;
    case Z: // bottom Row of M
      jac.row(2) = sin(y)*yd*jac_y - cos(y)*jac_yd;
      break;
    default:
      assert(false);
      break;
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
