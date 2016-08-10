/*
 * zmp_constraint.cc
 *
 *  Created on: May 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/constraint_container.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

typedef xpp::utils::MatVecVec MatVecVec;

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
ZmpConstraint::UpdateVariables (const ConstraintContainer* container)
{
  x_coeff_ = container->GetSplineCoefficients();
  supp_polygon_container_.SetFootholdsXY(container->GetFootholds());
}

ZmpConstraint::VectorXd
ZmpConstraint::EvaluateConstraint () const
{
  MatVecVec ineq = zmp_constraint_builder_.CalcZmpConstraints(supp_polygon_container_);
  return ineq.Mv.M*x_coeff_ + ineq.Mv.v ; //+ ineq.constant; // put into bound
}

ZmpConstraint::VecBound
ZmpConstraint::GetBounds () const
{
  std::vector<Bound> bounds;

  // evaluate once to get constant terms
  MatVecVec ineq = zmp_constraint_builder_.CalcZmpConstraints(supp_polygon_container_);

  for (int i=0; i<ineq.constant.rows(); ++i) {
    bounds.push_back(kInequalityBoundPositive_);
    bounds.at(i).lower_ -= ineq.constant[i];
  }
  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
