/**
 @file    centroidal_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#include <towr/models/centroidal_model.h>

#include <vector>

#include <xpp_states/cartesian_declarations.h>


namespace xpp {

CentroidalModel::CentroidalModel (double mass, const Eigen::Matrix3d& inertia,
                                  int ee_count)
    :DynamicModel(mass)
{
  SetCurrent(ComPos::Zero(), AngVel::Zero(), EELoad(ee_count), EEPos(ee_count));
  I_dense_ = inertia;
  I_       = inertia.sparseView();
  I_inv_   = inertia.inverse().sparseView();
}

CentroidalModel::~CentroidalModel ()
{
  // TODO Auto-generated destructor stub
}

CentroidalModel::BaseAcc
CentroidalModel::GetBaseAcceleration () const
{
  Vector3d f_lin, f_ang; f_lin.setZero(); f_ang.setZero();

  for (auto ee : ee_pos_.GetEEsOrdered()) {
    Vector3d f = ee_force_.at(ee);
    f_ang += f.cross(com_pos_-ee_pos_.at(ee));
    f_lin += f;
  }

  // moved gravity to bounds, as this is constant and would mess up SNOPT
  // static const Vector3d fg_W(0.0, 0.0, -m_*kGravity);
  // f_lin += fg_W;

  BaseAcc acc;
  acc.segment(AX, kDim3d) = I_inv_*(f_ang - omega_.cross(I_dense_*omega_)); // coriolis terms
  acc.segment(LX, kDim3d) = 1./m_ *f_lin;

  return acc;
}

// just a helper function
using Jacobian = DynamicModel::Jacobian;

static Jacobian
BuildCrossProductMatrix(const Vector3d& in)
{
  Jacobian out(3,3);

  out.coeffRef(0,1) = -in(2); out.coeffRef(0,2) =  in(1);
  out.coeffRef(1,0) =  in(2); out.coeffRef(1,2) = -in(0);
  out.coeffRef(2,0) = -in(1); out.coeffRef(2,1) =  in(0);

  return out;
}

Jacobian
CentroidalModel::GetJacobianOfAccWrtBaseLin (const Jacobian& jac_pos_base_lin) const
{
  // build the com jacobian
  int n = jac_pos_base_lin.cols();

  Jacobian jac_ang(kDim3d, n);
  for (const Vector3d& f : ee_force_.ToImpl()) {
    Jacobian jac_comp = BuildCrossProductMatrix(f)*jac_pos_base_lin;
    jac_ang += jac_comp;
  }

  Jacobian jac(kDim6d, n);
  jac.middleRows(AX, kDim3d) = I_inv_*jac_ang;

  // linear acceleration does not depend on base
  return jac;
}

Jacobian
CentroidalModel::GetJacobianOfAccWrtBaseAng (const Jacobian& jac_ang_vel) const
{
  int n = jac_ang_vel.cols();

  // the 6D base acceleration does not depend on base orientation, but on angular velocity
  // add derivative of w x Iw here!!!
  Jacobian jac_coriolis  = -BuildCrossProductMatrix(I_*omega_)*jac_ang_vel;
  jac_coriolis          +=  BuildCrossProductMatrix(omega_)*I_*jac_ang_vel;

  Jacobian jac(kDim6d, n);
  jac.middleRows(AX, kDim3d) = I_inv_*(-jac_coriolis);

  return jac;
}

Jacobian
CentroidalModel::GetJacobianofAccWrtForce (const Jacobian& ee_force_jac,
                                           EndeffectorID ee) const
{
  Vector3d r = com_pos_-ee_pos_.at(ee);
  Jacobian jac_ang = -BuildCrossProductMatrix(r)*ee_force_jac;

  int n = ee_force_jac.cols();
  Jacobian jac(kDim6d, n);
  jac.middleRows(AX, kDim3d) = I_inv_*jac_ang;
  jac.middleRows(LX, kDim3d) = 1./m_*ee_force_jac;

  return jac;
}

Jacobian
CentroidalModel::GetJacobianofAccWrtEEPos (const Jacobian& jac_ee_pos,
                                           EndeffectorID ee) const
{
  Vector3d f = ee_force_.at(ee);
  Jacobian jac_ang = BuildCrossProductMatrix(f)*(-jac_ee_pos);

  Jacobian jac(kDim6d, jac_ang.cols());
  jac.middleRows(AX, kDim3d) = I_inv_*jac_ang;
  // linear acceleration does not depend on endeffector position.
  return jac;
}

} /* namespace xpp */


