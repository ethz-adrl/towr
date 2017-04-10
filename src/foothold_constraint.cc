/**
 @file    a_foothold_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Defines an abstract FootholdCost class and one concrete derivation.
 */

#include <xpp/opt/constraints/foothold_constraint.h>
#include <xpp/opt/endeffectors_motion.h>

namespace xpp {
namespace opt {

FootholdConstraint::FootholdConstraint (const OptVarsPtr& opt_vars,
                                        const NominalStance& nom_W,
                                        double t)
{
  name_ = "Foothold Constraint";
  ee_motion_ = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetSet("endeffectors_motion"));
  desired_ee_pos_W_ = nom_W;
  t_ = t;

  int num_constraints = nom_W.GetCount() * kDim2d;
  SetDimensions(opt_vars->GetOptVarsVec(), num_constraints);

  // Jacobian doesn't change with values of optimization variables
  // only holds if t is during stance phase, otherwise Jacobian
  // dependent on time.
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_motion_->GetId());
  int k = 0;
  for (const auto ee : nom_W.GetEEsOrdered())
    for (auto dim : d2::AllDimensions)
      jac.row(k++) = ee_motion_->GetJacobianWrtOptParams(t,ee,dim);
}

FootholdConstraint::~FootholdConstraint ()
{
}

void
FootholdConstraint::UpdateConstraintValues ()
{
  int k=0;
  for (const auto& ee_state : ee_motion_->GetEndeffectorsVec(t_))
    for (auto dim : d2::AllDimensions)
      g_(k++) = ee_state.p(dim);
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

