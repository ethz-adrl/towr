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
                                        const Vector2d& body_xy,
                                        const NominalStance& nom)
{
  name_ = "Foothold Constraint";
  ee_motion_ = ee_motion;
  body_xy_ = body_xy;
  nominal_stance_ = nom;
  t_ = ee_motion->GetTotalTime();//0.0; // time at start

  int num_constraints = nom.GetEECount() * kDim2d;
  SetDependentVariables({ee_motion}, num_constraints);


  // jacobian doesn't change with values of optimization variables
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_motion_->GetID());
  int k = 0;
  for (auto c : ee_motion_->GetContacts(t_)) {
    for (auto dim : d2::AllDimensions) {
      int idx = ee_motion_->Index(c.ee,c.id,dim);
      jac.insert(k++,idx) =  -1;
    }
  }
}

FootholdConstraint::~FootholdConstraint ()
{
}

void
FootholdConstraint::UpdateConstraintValues ()
{
  int row=0;
  for (auto c : ee_motion_->GetContacts(t_)) {
    Vector2d p = c.p.topRows<kDim2d>();
    Vector2d foot_to_nominal_W = GetContactToNominalInWorld(p, c.ee);
    for (auto dim : d2::AllDimensions)
      g_(row++) = foot_to_nominal_W(dim);
  }
}

Vector2d
FootholdConstraint::GetContactToNominalInWorld (const Vector2d& foot_W, EndeffectorID ee) const
{
  Vector2d goal_to_nom_B = nominal_stance_.At(ee).topRows<kDim2d>();
  Eigen::Matrix2d W_R_B = Eigen::Matrix2d::Identity(); // attention: assumes no rotation world to base
  Vector2d goal_to_nom_W = W_R_B * goal_to_nom_B;

  Vector2d foot_to_goal_W    = body_xy_ - foot_W;
  Vector2d foot_to_nominal_W = foot_to_goal_W + goal_to_nom_W;

  return foot_to_nominal_W;
}

void
FootholdConstraint::UpdateBounds ()
{
  std::fill(bounds_.begin(), bounds_.end(), kEqualityBound_);
}

} /* namespace opt */
} /* namespace xpp */

