/**
 @file    a_foothold_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Defines an abstract FootholdCost class and one concrete derivation.
 */

#include <xpp/opt/constraints/foothold_constraint.h>

#include <string>
#include <vector>
#include <Eigen/Dense>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/bound.h>
#include <xpp/opt/variables/endeffectors_motion.h>

namespace xpp {
namespace opt {

FootholdConstraint::FootholdConstraint (const OptVarsPtr& opt_vars,
                                        const NominalStance& nom_W,
                                        double t)
{
  ee_motion_ = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetComponent("endeffectors_motion"));
  desired_ee_pos_W_ = nom_W;
  t_ = t;

  int num_constraints = nom_W.GetCount() * kDim2d;
  SetName("FootholdConstraint");
  SetRows(num_constraints);
  AddComposite(opt_vars);
}

FootholdConstraint::~FootholdConstraint ()
{
}

VectorXd
FootholdConstraint::GetValues () const
{
  VectorXd g(GetRows());
  int k=0;
  for (const auto& ee_state : ee_motion_->GetEndeffectors(t_).ToImpl())
    for (auto dim : d2::AllDimensions)
      g(k++) = ee_state.p(dim);

  return g;
}

VecBound
FootholdConstraint::GetBounds () const
{
  VecBound bounds;

  for (auto ee : desired_ee_pos_W_.GetEEsOrdered()) {
    Vector3d ee_pos_W = desired_ee_pos_W_.At(ee);
    for (auto dim : d2::AllDimensions)
      bounds.push_back(Bound(ee_pos_W(dim), ee_pos_W(dim)));
  }

  return bounds;
}

void
FootholdConstraint::FillJacobianWithRespectTo (std::string var_set,
                                              Jacobian& jac) const
{
  if (var_set == ee_motion_->GetName()) {
    int k = 0;
    for (const auto ee : desired_ee_pos_W_.GetEEsOrdered())
      for (auto dim : d2::AllDimensions)
        jac.row(k++) = ee_motion_->GetJacobianWrtOptParams(t_,ee,dim);
  }
}

} /* namespace opt */
} /* namespace xpp */

