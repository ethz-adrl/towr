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
#include <xpp/opt/variables/center_of_pressure.h>
#include <xpp/opt/variables/endeffector_load.h>
#include <xpp/opt/variables/endeffectors_motion.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint (const OptVarsPtr& opt_vars,
                                      double T,
                                      double dt)
    :TimeDiscretizationConstraint(T,dt)
{
  name_ = "Dynamic";

  com_motion_ = std::dynamic_pointer_cast<BaseMotion>        (opt_vars->GetSet("base_motion"));
  ee_motion_  = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetSet("endeffectors_motion"));
  ee_load_    = std::dynamic_pointer_cast<EndeffectorLoad>   (opt_vars->GetSet("endeffector_load"));

  cop_        = std::dynamic_pointer_cast<CenterOfPressure>  (opt_vars->GetSet("center_of_pressure"));
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
  auto com     = com_motion_->GetCom(t);
  auto ee_load = ee_load_->GetLoadValues(t);
  auto ee_pos  = ee_motion_->GetEndeffectorsPos(t);
  model_.SetCurrent(com.p, kHeight_, ee_load, ee_pos);

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
  auto ee_load = ee_load_->GetLoadValues(t);
  auto ee_pos  = ee_motion_->GetEndeffectorsPos(t);
  model_.SetCurrent(com.p, kHeight_, ee_load, ee_pos);

  Vector2d cop = cop_->GetCop(t);

  for (auto dim : d2::AllDimensions) {
    int row = GetRow(k,dim);

    // endeffector load
    double jac_model =  model_.GetDerivativeOfAccWrtCop(dim);
    jac_cop.row(row) = jac_model*cop_->GetJacobianWrtCop(t,dim);


    // endeffector position


    // base motion
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
