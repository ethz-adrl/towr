/**
 @file    range_of_motion_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/zmp/range_of_motion_cost.h>

namespace xpp {
namespace zmp {

RangeOfMotionCost::RangeOfMotionCost (OptimizationVariables& subject)
{
  subject_ = &subject;
  subject_->RegisterObserver(this);
}

void
RangeOfMotionCost::Init (const ContinuousSplineContainer& spline,
                               const SupportPolygonContainer& support)
{
  continuous_spline_container_ = spline;
  supp_polygon_container_ = support;

  Update();
}

void
RangeOfMotionCost::Update ()
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

double
RangeOfMotionCost::EvaluateCost () const
{
  VectorXd distances = builder_.DistanceToNominalStance(continuous_spline_container_,
                                                        supp_polygon_container_);
  return distances.norm();
}

} /* namespace zmp */
} /* namespace xpp */
