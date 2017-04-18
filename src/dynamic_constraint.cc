/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/dynamic_constraint.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/bound.h>
#include <xpp/constraint.h>
#include <xpp/opt/variables/base_motion.h>
#include <xpp/opt/variables/endeffector_load.h>
#include <xpp/opt/variables/endeffectors_motion.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint (const OptVarsPtr& opt_vars,
                                      double T,
                                      double dt)
    :TimeDiscretizationConstraint(T,dt, kDim2d)
{
  name_ = "Dynamic";

  com_motion_ = std::dynamic_pointer_cast<BaseMotion>        (opt_vars->GetSet("base_motion"));
  ee_motion_  = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetSet("endeffectors_motion"));
  ee_load_    = std::dynamic_pointer_cast<EndeffectorLoad>   (opt_vars->GetSet("endeffector_load"));

  // zmp_ DRY with number of constraints in BaseClass constructor
  int num_constraints = GetNumberOfNodes()*kDim2d;
  SetDimensions(opt_vars->GetOptVarsVec(), num_constraints);
}

DynamicConstraint::~DynamicConstraint ()
{
}

void
DynamicConstraint::UpdateConstraintAtInstance(double t, int k) const
{
  auto com     = com_motion_->GetCom(t);
  auto ee_load = ee_load_   ->GetLoadValues(t);
  auto ee_pos  = ee_motion_ ->GetEndeffectorsPos(t);
  model_.SetCurrent(com.p, com_motion_->GetZHeight(), ee_load, ee_pos);

  // acceleration as predefined by physics
  Vector2d acc_physics = model_.GetAcceleration();
  for (auto dim : d2::AllDimensions)
    g_new_(GetRow(k,dim)) = acc_physics(dim) - com.a(dim);
}

void
DynamicConstraint::UpdateBoundsAtInstance(double t, int k)
{
  for (auto dim : d2::AllDimensions)
    bounds_.at(GetRow(k,dim)) = kEqualityBound_;
}

void
DynamicConstraint::UpdateJacobianAtInstance(double t, int k)
{
  Jacobian& jac_com  = GetJacobianRefWithRespectTo(com_motion_->GetId());
  Jacobian& jac_load = GetJacobianRefWithRespectTo(ee_load_->GetId());
  Jacobian& jac_ee   = GetJacobianRefWithRespectTo(ee_motion_->GetId());

  auto com     = com_motion_->GetCom(t);
  auto ee_load = ee_load_   ->GetLoadValues(t);
  auto ee_pos  = ee_motion_ ->GetEndeffectorsPos(t);
  model_.SetCurrent(com.p, com_motion_->GetZHeight(), ee_load, ee_pos);

  for (auto dim : d2::AllDimensions) {
    int row = GetRow(k,dim);

    for (auto ee : ee_load.GetEEsOrdered()) {

      // w.r.t endeffector load
      double deriv_load = model_.GetDerivativeOfAccWrtLoad(ee, dim);
      jac_load.coeffRef(row, ee_load_->Index(t,ee)) = deriv_load;

      // w.r.t endeffector position
      double deriv_ee = model_.GetDerivativeOfAccWrtEEPos(ee);
      // zmp_ this is ugly, DRY, fix!!!!
      // Somehow generalize access to jacobians and clearly understand
      // update sequence.
      if (ee == E0) // overwrite jacobian
        jac_ee.row(row) = deriv_ee* ee_motion_->GetJacobianWrtOptParams(t, ee, dim);
      else // append
        jac_ee.row(row) += deriv_ee* ee_motion_->GetJacobianWrtOptParams(t, ee, dim);
    }


    // w.r.t base motion
    Coords3D dim3d = static_cast<Coords3D>(dim);
    Jacobian jac_acc     = com_motion_->GetJacobian(t, kAcc, dim3d);
    Jacobian jac_physics = model_.GetJacobianWrtBase(*com_motion_, t, dim3d);
    jac_com.row(row) = jac_physics - jac_acc;
  }
}

int
DynamicConstraint::GetRow (int node, int dimension) const
{
  return kDim2d*node + dimension;
}

} /* namespace opt */
} /* namespace xpp */
