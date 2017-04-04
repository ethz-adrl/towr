/**
 @file    a_foothold_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Defines an abstract FootholdCost class and one concrete derivation.
 */

#include <xpp/opt/constraints/foothold_constraint.h>

namespace xpp {
namespace opt {

FootholdConstraint::FootholdConstraint (const EEMotionPtr& ee_motion,
                                        const NominalStance& nom_W,
                                        double t)
{
  name_ = "Foothold Constraint";
  ee_motion_ = ee_motion;
  desired_ee_pos_W_ = nom_W;
  t_ = t;

  int num_constraints = nom_W.GetCount() * kDim2d;
  SetDependentVariables({ee_motion}, num_constraints);

  // jacobian doesn't change with values of optimization variables
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_motion_->GetID());
  int k = 0;
  for (auto c : ee_motion_->GetContacts(t)) {
    for (auto dim : d2::AllDimensions) {
      int idx = ee_motion_->Index(c.ee,c.id,dim);
      jac.insert(k++,idx) = 1.0;
    }
  }
}

FootholdConstraint::~FootholdConstraint ()
{
}

void
FootholdConstraint::UpdateConstraintValues ()
{
  int k=0;
  for (auto c : ee_motion_->GetContacts(t_))
    for (auto dim : d2::AllDimensions)
      g_(k++) = c.p(dim);
}

void
FootholdConstraint::UpdateBounds ()
{
  int k=0;
  for (auto ee : desired_ee_pos_W_.GetEEsOrdered()) {
    Vector3d ee_pos_W = desired_ee_pos_W_.At(ee);
    for (auto dim : d2::AllDimensions)
      bounds_.at(k++) = Bound(ee_pos_W(dim), ee_pos_W(dim));
  }
}

} /* namespace opt */
} /* namespace xpp */

