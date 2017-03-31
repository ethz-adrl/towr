/**
 @file    support_area_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Defines the DynamicConstraint class
 */

#include <xpp/opt/constraints/support_area_constraint.h>

namespace xpp {
namespace opt {

SupportAreaConstraint::SupportAreaConstraint (const EEMotionPtr& ee_motion,
                                              const EELoadPtr& ee_load,
                                              const CopPtr& cop,
                                              double dt)
    :TimeDiscretizationConstraint(ee_motion->GetTotalTime(), dt)
{
  name_ = "Support Area";
  ee_motion_ = ee_motion;
  ee_load_ = ee_load;
  cop_ = cop;

  int num_constraints = GetNumberOfNodes()*kDim2d;
  SetDependentVariables({ee_motion, ee_load, cop}, num_constraints);
}

SupportAreaConstraint::~SupportAreaConstraint ()
{
}

int
SupportAreaConstraint::GetRow (int node, int dimension) const
{
  return node*kDim2d + dimension;
}

void
SupportAreaConstraint::UpdateConstraintAtInstance (double t, int k)
{
  Vector2d convex_contacts = Vector2d::Zero();
  auto lambda_k = ee_load_->GetLoadValues(t);

  // spring_clean_ could actually also be all the endeffectors, then contact flags would only
  // be in other constraint
//  auto ee_state = ee_motion_->GetEndeffectors(t);
//  for (auto ee : ee_state.GetEEsOrdered())
//    convex_contacts += lambda_k.At(ee)*ee_state.At(ee).Get2D().p;

  for (auto f : ee_motion_->GetContacts(t))
    convex_contacts += lambda_k.At(f.ee)*f.p.topRows<kDim2d>();

  Vector2d cop = cop_->GetCop(t);
  for (auto dim : d2::AllDimensions)
    g_(GetRow(k,dim)) = convex_contacts(dim) - cop(dim);
}

void
SupportAreaConstraint::UpdateBoundsAtInstance (double t, int k)
{
  for (auto dim : d2::AllDimensions)
    bounds_.at(GetRow(k,dim)) = kEqualityBound_;
}

void
SupportAreaConstraint::UpdateJacobianAtInstance (double t, int k)
{
  UpdateJacobianWithRespectToLoad(t,k);
  UpdateJacobianWithRespectToEEMotion(t,k);
  UpdateJacobianWithRespectToCop(t,k); // actually constant, so doesn't have to be here
}

void
SupportAreaConstraint::UpdateJacobianWithRespectToLoad(double t, int k)
{
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_load_->GetID());

  for (auto c : ee_motion_->GetContacts(t)) {
    for (auto dim : d2::AllDimensions) {
      int idx = ee_load_->Index(t,c.ee);
      jac.coeffRef(GetRow(k,dim), idx) = c.p(dim);
    }
  }
}

void
SupportAreaConstraint::UpdateJacobianWithRespectToEEMotion (double t, int k)
{
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_motion_->GetID());

  auto lambda_k = ee_load_->GetLoadValues(t);

  // zmp_ use this instead of below
//  auto ee_state = ee_motion_->GetEndeffectors(t);
//  for (auto ee : ee_state.GetEEsOrdered()) {
//    for (auto dim : d2::AllDimensions) {
//      int idx = ee_motion_->Index(ee, c.id, dim);
//      jac.coeffRef(GetRow(k,dim), idx) = lambda_k.At(ee);
//    }
//  }

  for (auto c : ee_motion_->GetContacts(t)) {
    for (auto dim : d2::AllDimensions) {
      int idx = ee_motion_->Index(c.ee, c.id, dim);
      jac.coeffRef(GetRow(k,dim), idx) = lambda_k.At(c.ee);
    }
  }
}

void
SupportAreaConstraint::UpdateJacobianWithRespectToCop (double t, int k)
{
  Jacobian& jac = GetJacobianRefWithRespectTo(cop_->GetID());

  for (auto dim : d2::AllDimensions)
    jac.row(GetRow(k,dim)) = -1 * cop_->GetJacobianWrtCop(t,dim);
}

} /* namespace opt */
} /* namespace xpp */
