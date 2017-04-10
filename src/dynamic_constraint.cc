/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/dynamic_constraint.h>
#include <xpp/opt/base_motion.h>
#include <xpp/opt/center_of_pressure.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint (const OptVarsPtr& opt_vars,
                                      double T,
                                      double dt)
    :TimeDiscretizationConstraint(T,dt)
{
  kHeight_ = 0.0;
  name_ = "Dynamic";

  com_motion_ = std::dynamic_pointer_cast<BaseMotion>      (opt_vars->GetSet("base_motion"));
  cop_        = std::dynamic_pointer_cast<CenterOfPressure>(opt_vars->GetSet("center_of_pressure"));
  kHeight_ = com_motion_->GetZHeight();

  int num_constraints = GetNumberOfNodes()*kDim2d;
  SetDimensions(opt_vars->GetOptVarsVec(), num_constraints);
}

DynamicConstraint::~DynamicConstraint ()
{
}

void
DynamicConstraint::UpdateConstraintAtInstance(double t, int k)
{
  auto com = com_motion_->GetCom(t);
  model_.SetCurrent(com.p, com.v, kHeight_);

  // acceleration as predefined by physics
  Vector2d acc_physics = model_.GetDerivative(cop_->GetCop(t));
  for (auto dim : d2::AllDimensions)
    g_(GetRow(k,dim)) = acc_physics(dim) - com.a(dim);
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
  Jacobian& jac_cop = GetJacobianRefWithRespectTo(cop_->GetId());
  Jacobian& jac_com = GetJacobianRefWithRespectTo(com_motion_->GetId());

  auto com = com_motion_->GetCom(t);
  model_.SetCurrent(com.p, com.v, kHeight_);
  Vector2d cop = cop_->GetCop(t);

  for (auto dim : d2::AllDimensions) {
    int row = GetRow(k,dim);

    double jac_model =  model_.GetJacobianApproxWrtCop(dim);
    jac_cop.row(row) = jac_model*cop_->GetJacobianWrtCop(t,dim);

    Coords3D dim3d = static_cast<Coords3D>(dim);
    Jacobian jac_acc     = com_motion_->GetJacobian(t, kAcc, dim3d);
    Jacobian jac_physics = model_.GetJacobianApproxWrtSplineCoeff(*com_motion_, t, dim3d, cop);
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
