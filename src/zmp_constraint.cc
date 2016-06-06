/*
 * zmp_constraint.cc
 *
 *  Created on: May 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint.h>

namespace xpp {
namespace zmp {

ZmpConstraint::ZmpConstraint (OptimizationVariables& subject)
    :IObserver(subject)
{
}

void
ZmpConstraint::Init (const ContinuousSplineContainer& spline_container,
                     const SupportPolygonContainer& supp_polygon_container,
                     double walking_height)
{
  supp_polygon_container_ = supp_polygon_container;
  zmp_constraint_builder_.Init(spline_container, walking_height);

  Update();
}

void
ZmpConstraint::Update ()
{
  x_coeff_ = subject_->GetSplineCoefficients();
  footholds_ = subject_->GetFootholdsStd();

  // fixme move this to foothold class and generally see if i really need
  // the previous support polygon container, or if footholds + legs is enough
  // for sure need start stance
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
