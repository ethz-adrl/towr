/**
 @file    centroidal_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#include "xpp/opt/centroidal_model.h"

namespace xpp {
namespace opt {

CentroidalModel::CentroidalModel ()
{
  // rough values for hyq
  m_ = 80;
  I_ = buildInertiaTensor( 1.209488,
                           5.5837,
                           6.056973,
                           0.00571,
                           -0.190812,
                           -0.012668);

  I_inv_ = I_.inverse().sparseView(1.0, -1.0); // don't treat zeros as sparse
}

CentroidalModel::~CentroidalModel ()
{
  // TODO Auto-generated destructor stub
}

CentroidalModel::BaseAcc
CentroidalModel::GetBaseAcceleration () const
{
  Vector3d f_lin, ang; f_lin.setZero(); ang.setZero();

  for (auto ee : ee_pos_.GetEEsOrdered()) {
    Vector3d f = ee_force_.At(ee);
    ang += f.cross(com_pos_-ee_pos_.At(ee));
    f_lin += f;
  }

  // moved gravity to bounds, as this is constant and would mess up SNOPT
  // static const Vector3d fg_W(0.0, 0.0, -m_*kGravity);
  // f_lin += fg_W;

  BaseAcc acc;
  acc.segment(AX, kDim3d) = I_inv_*ang;
  acc.segment(LX, kDim3d) = 1./m_ *f_lin;

  return acc;
}

Jacobian
CentroidalModel::GetJacobianOfAccWrtBaseLin (const BaseLin& base_lin,
                                             double t) const
{
  // build the com jacobian
  int n = base_lin.GetRows();

  Jacobian jac_ang(kDim3d, n);
  for (const Vector3d& f : ee_force_.ToImpl())
    jac_ang += BuildCrossProductMatrix(f)*base_lin.GetJacobian(t, kPos);

  Jacobian jac(kDim6d, n);
  jac.middleRows(AX, kDim3d) = I_inv_*jac_ang;

  // linear acceleration does not depend on base
  return jac;
}

Jacobian
CentroidalModel::GetJacobianOfAccWrtBaseAng (const BaseAng& base_ang,
                                             double t) const
{
  // the 6D base acceleration does not depend on base orientation
  return Jacobian(kDim6d, base_ang.GetRows());
}

Jacobian
CentroidalModel::GetJacobianofAccWrtForce (const EndeffectorsForce& ee_force,
                                          double t,
                                          EndeffectorID ee) const
{
  int n = ee_force.GetRows();
  Jacobian ee_force_jac(kDim3d, n);
  for (auto dim : {X,Y,Z})
    ee_force_jac.row(dim) = ee_force.GetJacobian(t, ee, dim);

  Jacobian jac_lin = ee_force_jac;

  Vector3d r = com_pos_-ee_pos_.At(ee);
  Jacobian jac_ang = -BuildCrossProductMatrix(r)*ee_force_jac;


  Jacobian jac(kDim6d, n);
  jac.middleRows(AX, kDim3d) = I_inv_*jac_ang;
  jac.middleRows(LX, kDim3d) = 1./m_*jac_lin;

  return jac;
}

Jacobian
CentroidalModel::GetJacobianofAccWrtEEPos (const Jacobian& jac_ee_pos,
                                           EndeffectorID ee) const
{
  Vector3d f = ee_force_.At(ee);
  Jacobian jac_ang = BuildCrossProductMatrix(f)*(-jac_ee_pos);

  Jacobian jac(kDim6d, jac_ang.cols());
  jac.middleRows(AX, kDim3d) = I_inv_*jac_ang;
  // linear acceleration does not depend on endeffector position.
  return jac;
}


} /* namespace opt */
} /* namespace xpp */
