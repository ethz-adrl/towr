/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/dynamic_constraint.h>

#include <Eigen/Dense>
#include <vector>

#include <xpp/state.h>

#include <xpp/opt/lip_model.h>
#include <xpp/opt/centroidal_model.h>
#include <xpp/opt/variables/base_motion.h>
#include <xpp/opt/variables/endeffectors_force.h>
#include <xpp/opt/variables/endeffectors_motion.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint (const OptVarsPtr& opt_vars,
                                      double T,
                                      double dt)
    :TimeDiscretizationConstraint(T, dt, opt_vars)
{
  model_ = std::make_shared<CentroidalModel>();
//  model_ = std::make_shared<LIPModel>();

  SetName("DynamicConstraint");
  base_motion_ = std::dynamic_pointer_cast<BaseMotion>        (opt_vars->GetComponent("base_motion"));
  ee_motion_   = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetComponent("endeffectors_motion"));
  ee_load_     = std::dynamic_pointer_cast<EndeffectorsForce> (opt_vars->GetComponent("endeffector_force"));

  SetRows(GetNumberOfNodes()*kDim6d);
}

int
DynamicConstraint::GetRow (int node, Coords6D dimension) const
{
  return kDim6d*node + dimension;
}

DynamicConstraint::~DynamicConstraint ()
{
}

void
DynamicConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{
  // acceleration the system should have given by physics
  UpdateModel(t);
  Vector6d acc_model = model_->GetBaseAcceleration();

  // acceleration base has with current values of optimization variables
  // angular acceleration fixed to zero for now
  Vector6d acc_parametrization = base_motion_->GetBase(t).Get6dAcc();

  for (auto dim : AllDim6D)
    g(GetRow(k,dim)) = acc_model(dim) - acc_parametrization(dim);
}

void
DynamicConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{
  for (auto dim : AllDim6D) {
    if (dim == LZ)
      bounds.at(GetRow(k,dim)) = Bound(kGravity, kGravity);
    else
      bounds.at(GetRow(k,dim)) = kEqualityBound_;
  }
}

void
DynamicConstraint::UpdateJacobianAtInstance(double t, int k, Jacobian& jac,
                                            std::string var_set) const
{
  UpdateModel(t);

  int row = GetRow(k,AX);

  for (auto ee : model_->GetEEIDs()) {
    if (var_set == ee_load_->GetName())
      jac.middleRows(row, kDim6d) += model_->GetJacobianofAccWrtForce(*ee_load_, t, ee);

    if (var_set == ee_motion_->GetName())
      jac.middleRows(row, kDim6d) += model_->GetJacobianofAccWrtEEPos(*ee_motion_, t, ee);
  }

  if (var_set == base_motion_->GetName()) {
    Jacobian jac_model           = model_->GetJacobianOfAccWrtBase(*base_motion_, t);
    Jacobian jac_parametrization = base_motion_->GetJacobian(t, kAcc);

    jac.middleRows(row, kDim6d) = jac_model - jac_parametrization;
  }
}

void
DynamicConstraint::UpdateModel (double t) const
{
  auto com     = base_motion_->GetBase(t).lin.p_;
  auto ee_load = ee_load_   ->GetForce(t);
  auto ee_pos  = ee_motion_ ->GetEndeffectors(t).GetPos();
  model_->SetCurrent(com, ee_load, ee_pos);
}

} /* namespace opt */
} /* namespace xpp */

