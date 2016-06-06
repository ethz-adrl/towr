/*
 * range_of_motion_constraint.cc
 *
 *  Created on: May 26, 2016
 *      Author: winklera
 */

#include <xpp/zmp/range_of_motion_constraint.h>

namespace xpp {
namespace zmp {

RangeOfMotionConstraint::RangeOfMotionConstraint (OptimizationVariables& subject)
    :IObserver(subject)
{
}

void
RangeOfMotionConstraint::Init (const ContinuousSplineContainer& spline,
                               const SupportPolygonContainer& support)
{
  continuous_spline_container_ = spline;
  supp_polygon_container_ = support;

  Update();
}

void
RangeOfMotionConstraint::Update ()
{
  VectorXd x_coeff      = subject_->GetSplineCoefficients();
  FootholdsXY footholds = subject_->GetFootholdsStd();

  continuous_spline_container_.AddOptimizedCoefficients(x_coeff);
  // fixme move this to foothold class and generally see if i really need
  // the previous support polygon container, or if footholds + legs is enough
  // for sure need start stance
  for (uint i=0; i<footholds.size(); ++i)
    supp_polygon_container_.SetFootholdsXY(i,footholds.at(i).x(), footholds.at(i).y());
}

RangeOfMotionConstraint::VectorXd
RangeOfMotionConstraint::EvaluateConstraint () const
{
  return builder_.DistanceToNominalStance(continuous_spline_container_,
                                          supp_polygon_container_);
}

RangeOfMotionConstraint::VecBound
RangeOfMotionConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  VectorXd g = EvaluateConstraint();
  double radius = 0.2; //m
  Bound bound(-radius, +radius);

  for (int i=0; i<g.rows(); ++i)
    bounds.push_back(bound);

  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
