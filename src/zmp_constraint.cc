/*
 * zmp_constraint.cc
 *
 *  Created on: May 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

using MatVecVec = xpp::utils::MatVecVec;
using MatVec = xpp::utils::MatVec;

ZmpConstraint::ZmpConstraint ()
{
}

void
ZmpConstraint::Init (const OptimizationVariablesInterpreter& interpreter)
{
  supp_polygon_container_ = interpreter.GetSuppPolygonContainer();
  zmp_constraint_builder_.Init(interpreter.GetSplineStructure(), interpreter.GetRobotHeight());
}

void
ZmpConstraint::UpdateVariables (const OptimizationVariables* subject)
{
  x_coeff_           = subject->GetVariables(VariableNames::kSplineCoeff);
  VectorXd footholds = subject->GetVariables(VariableNames::kFootholds);
  supp_polygon_container_.SetFootholdsXY(utils::ConvertEigToStd(footholds));

  coeff_and_footholds_ = subject->GetOptimizationVariables();
  zmp_constraint_builder_.spline_structure_->SetCoefficients(x_coeff_);
}

ZmpConstraint::VectorXd
ZmpConstraint::EvaluateConstraint () const
{
//  MatVec constraint_approx = zmp_constraint_builder_.GetJacobian(supp_polygon_container_);
//  return constraint_approx.v;
//  return constraint_approx.M*coeff_and_footholds_ + constraint_approx.v;

  return zmp_constraint_builder_.GetDistanceToLineMargin(supp_polygon_container_);

//  MatVecVec ineq = zmp_constraint_builder_.CalcZmpConstraints(supp_polygon_container_);
//  return ineq.Mv.M*x_coeff_ + ineq.Mv.v ; //+ ineq.constant; // put into bound
}

ZmpConstraint::VecBound
ZmpConstraint::GetBounds () const
{
  std::vector<Bound> bounds;

  // evaluate once to get constant terms
  MatVecVec ineq = zmp_constraint_builder_.CalcZmpConstraints(supp_polygon_container_);

  for (int i=0; i<ineq.constant.rows(); ++i) {
    bounds.push_back(kInequalityBoundPositive_);
//    bounds.at(i).lower_ -= ineq.constant[i];
  }
  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
