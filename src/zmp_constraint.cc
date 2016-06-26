/*
 * zmp_constraint.cc
 *
 *  Created on: May 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

ZmpConstraint::ZmpConstraint (OptimizationVariables& subject)
    :IObserver(subject)
{
}

void
ZmpConstraint::Init (const OptimizationVariablesInterpreter& interpreter)
{
  supp_polygon_container_ = interpreter.GetSuppPolygonContainer();
  zmp_constraint_builder_.Init(interpreter.GetSplineStructure(), interpreter.GetRobotHeight());

  Update();
}

void
ZmpConstraint::Update ()
{
  x_coeff_ = subject_->GetSplineCoefficients();
  footholds_ = subject_->GetFootholdsStd();

  // fixme use interpreter class for this, it knows about the foothold sequence
  for (uint i=0; i<footholds_.size(); ++i)
    supp_polygon_container_.SetFootholdsXY(i,footholds_.at(i).x(), footholds_.at(i).y());
}

ZmpConstraint::VectorXd
ZmpConstraint::EvaluateConstraint () const
{
  MatVec ineq = zmp_constraint_builder_.CalcZmpConstraints(supp_polygon_container_);
  return ineq.M*x_coeff_ + ineq.v;
}

ZmpConstraint::VecBound
ZmpConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  VectorXd g = EvaluateConstraint(); // only need the number of constraints

  for (int i=0; i<g.rows(); ++i)
    bounds.push_back(kInequalityBoundPositive_);

  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
