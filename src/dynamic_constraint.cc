/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/dynamic_constraint.h>
#include <xpp/opt/base_motion.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint (const BaseMotionPtr& com_motion,
                                      const CopPtr& cop,
                                      double T,
                                      double dt)
    :TimeDiscretizationConstraint(T,dt)
{
  kHeight_ = 0.0;
  name_ = "Dynamic";

  com_motion_ = com_motion;
  cop_ = cop;
  kHeight_ = com_motion->GetZHeight();

  int num_constraints = GetNumberOfNodes()*kDim2d;
  SetDependentVariables({com_motion, cop}, num_constraints);
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
  g_.middleRows<kDim2d>(kDim2d*k) = acc_physics - com.a;
}

void
DynamicConstraint::UpdateBoundsAtInstance(double t, int k)
{
  for (auto dim : d2::AllDimensions)
    bounds_.at(kDim2d*k + dim) = kEqualityBound_;
}

void
DynamicConstraint::UpdateJacobianAtInstance(double t, int k)
{
  Jacobian& jac_cop = GetJacobianRefWithRespectTo(cop_->GetID());
  Jacobian& jac_com = GetJacobianRefWithRespectTo(com_motion_->GetID());

  auto com = com_motion_->GetCom(t);
  model_.SetCurrent(com.p, com.v, kHeight_);
  Vector2d cop = cop_->GetCop(t);

  for (auto dim : d2::AllDimensions) {
    int row = kDim2d*k + dim;

    double jac_model =  model_.GetJacobianApproxWrtCop(dim);
    jac_cop.row(row) = jac_model*cop_->GetJacobianWrtCop(t,dim);

    Coords3D dim3d = static_cast<Coords3D>(dim);
    Jacobian jac_acc     = com_motion_->GetJacobian(t, kAcc, dim3d);
    Jacobian jac_physics = model_.GetJacobianApproxWrtSplineCoeff(*com_motion_, t, dim3d, cop);
    jac_com.row(row) = jac_physics - jac_acc;
  }
}

} /* namespace opt */
} /* namespace xpp */
