/*
 * range_of_motion_constraint.cc
 *
 *  Created on: May 26, 2016
 *      Author: winklera
 */

#include <xpp/zmp/range_of_motion_constraint.h>

#include <xpp/zmp/problem_specification.h> // fixme violates SRP, fix or rename

namespace xpp {
namespace zmp {

RangeOfMotionConstraint::RangeOfMotionConstraint (OptimizationVariables& subject)
{
  subject_ = &subject;
  subject_->RegisterObserver(this);
}

void
RangeOfMotionConstraint::Init (const ContinuousSplineContainer& c,
                               const SupportPolygonContainer& s)
{
  continuous_spline_container_ = c;
  supp_polygon_container_ = s;

  Update();
}

void
RangeOfMotionConstraint::Update ()
{
  x_coeff_ = subject_->GetSplineCoefficients();
  footholds_ = subject_->GetFootholds();

  continuous_spline_container_.AddOptimizedCoefficients(x_coeff_);
  // fixme move this to foothold class and generally see if i really need
  // the previous support polygon container, or if footholds + legs is enough
  // for sure need start stance
  for (uint i=0; i<footholds_.size(); ++i)
    supp_polygon_container_.SetFootholdsXY(i,footholds_.at(i).x(), footholds_.at(i).y());

}

RangeOfMotionConstraint::VectorXd
RangeOfMotionConstraint::EvaluateConstraint () const
{
  return ProblemSpecification::DistanceFootToNominalStance(supp_polygon_container_,
                                                           continuous_spline_container_);
}

RangeOfMotionConstraint::VecBound
RangeOfMotionConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  VectorXd g = EvaluateConstraint();
  Bound bound(-0.20, 0.20);

  for (int i=0; i<g.rows(); ++i)
    bounds.push_back(bound);

  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
