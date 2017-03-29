/**
 @file    a_foothold_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Defines an abstract FootholdCost class and one concrete derivation.
 */

#include <xpp/opt/a_foothold_constraint.h>
#include <xpp/opt/variable_names.h>

namespace xpp {
namespace opt {

AFootholdConstraint::AFootholdConstraint ()
{
  // TODO Auto-generated constructor stub
}

AFootholdConstraint::~AFootholdConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
AFootholdConstraint::Init (const EndeffectorsMotion& ee_motion)
{
  ee_motion_ = ee_motion;
}

void
AFootholdConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);
  ee_motion_.SetOptimizationParameters(footholds);
}

FootholdFinalStanceConstraint::FootholdFinalStanceConstraint (
                                    const EndeffectorsMotion& ee_motion,
                                    const Vector2d& goal_xy,
                                    const NominalStance& nom)
{
  Init(motion_structure);
  name_ = "Final Stance";
  goal_xy_ = goal_xy;
  nominal_stance_ = nom;
  final_free_contacts_ = motion_structure.GetPhases().back().contacts_opt_;
}

FootholdFinalStanceConstraint::~FootholdFinalStanceConstraint ()
{
}

FootholdFinalStanceConstraint::VectorXd
FootholdFinalStanceConstraint::UpdateConstraintValues () const
{
  VectorXd g(final_free_contacts_.size()*kDim2d);

  int row=0;
  for (auto c : final_free_contacts_) {
    Vector2d p = footholds_.at(c.id);
    Vector2d foot_to_nominal_W = GetContactToNominalInWorld(p, c.ee);
    for (auto dim : {X,Y})
      g(row++) = foot_to_nominal_W(dim);
  }

  return g;
}

FootholdFinalStanceConstraint::Vector2d
FootholdFinalStanceConstraint::GetContactToNominalInWorld (const Vector2d& foot_W, EndeffectorID ee) const
{
  Vector2d goal_to_nom_B = nominal_stance_.at(ee);
  Eigen::Matrix2d W_R_B = Eigen::Matrix2d::Identity(); // attention: assumes no rotation world to base
  Vector2d goal_to_nom_W = W_R_B * goal_to_nom_B;

  Vector2d foot_to_goal_W    = goal_xy_ - foot_W;
  Vector2d foot_to_nominal_W = foot_to_goal_W + goal_to_nom_W;

  return foot_to_nominal_W;
}

FootholdFinalStanceConstraint::Jacobian
FootholdFinalStanceConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac;

  if (var_set == VariableNames::kFootholds) {

    int n = footholds_.size() * kDim2d;
    int m = GetNumberOfConstraints();
    jac = Jacobian(m, n);

    int rows=0;
    for (auto c : final_free_contacts_) {
      for (auto dim : {X,Y}) {
        int idx = ContactVars::Index(c.id,dim);
        jac.insert(rows++,idx) =  -1;
      }
    }
  }
  return jac;
}

VecBound
FootholdFinalStanceConstraint::GetBounds () const
{
  int n_constraints = final_free_contacts_.size() * kDim2d;
  VecBound bounds(n_constraints);
  for (auto& bound : bounds)
    bound = kEqualityBound_;

  return bounds;
}

} /* namespace opt */
} /* namespace xpp */

