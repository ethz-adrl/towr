/*
 * zmp_constraint.cc
 *
 *  Created on: May 25, 2016
 *      Author: winklera
 */

#include <xpp/opt/zmp_constraint.h>
#include <xpp/opt/optimization_variables.h>

namespace xpp {
namespace opt {

ZmpConstraint::ZmpConstraint ()
{
}

void
ZmpConstraint::Init (const MotionStructure& structure,
                     const ComMotion& com_motion,
                     const Contacts& contacts,
                     double walking_height,
                     double dt)
{
  zmp_constraint_builder_.Init(structure, com_motion, contacts, walking_height, dt);
}

void
ZmpConstraint::UpdateVariables (const OptimizationVariables* subject)
{
  VectorXd x_coeff   = subject->GetVariables(VariableNames::kSplineCoeff);
  VectorXd footholds = subject->GetVariables(VariableNames::kFootholds);

  zmp_constraint_builder_.Update(x_coeff, footholds);
}

ZmpConstraint::VectorXd
ZmpConstraint::EvaluateConstraint () const
{
//  MatVec constraint_approx = zmp_constraint_builder_.GetJacobian(supp_polygon_container_);
//  return constraint_approx.v;
//  return constraint_approx.M*coeff_and_footholds_ + constraint_approx.v;


  // write update function for zmp_constraint_builder
  return zmp_constraint_builder_.GetDistanceToLineMargin();

//  MatVecVec ineq = zmp_constraint_builder_.CalcZmpConstraints(supp_polygon_container_);
//  return ineq.Mv.M*x_coeff_ + ineq.Mv.v ; //+ ineq.constant; // put into bound
}

ZmpConstraint::VecBound
ZmpConstraint::GetBounds () const
{
  std::vector<Bound> bounds;

  VectorXd d = zmp_constraint_builder_.GetDistanceToLineMargin();

//  // evaluate once to get constant terms
//  MatVecVec ineq = zmp_constraint_builder_.CalcZmpConstraints(supp_polygon_container_);

  for (int i=0; i<d.rows(); ++i) {
    bounds.push_back(kInequalityBoundPositive_);
//    bounds.at(i).lower_ -= ineq.constant[i];
  }
  return bounds;
}

ZmpConstraint::Jacobian
ZmpConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  if (var_set == VariableNames::kSplineCoeff)
    jac =  zmp_constraint_builder_.GetJacobianWrtMotion();

  if (var_set == VariableNames::kFootholds)
    jac = zmp_constraint_builder_.GetJacobianWrtContacts();

  return jac;
}

} /* namespace zmp */
} /* namespace xpp */
