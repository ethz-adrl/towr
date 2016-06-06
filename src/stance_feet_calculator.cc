/**
 @file    stance_feet_calculator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/zmp/stance_feet_calculator.h>

#include <xpp/zmp/spline_container.h>

namespace xpp {
namespace zmp {

StanceFeetCalculator::StanceFeetCalculator ()
{
  // TODO Auto-generated constructor stub
}

StanceFeetCalculator::~StanceFeetCalculator ()
{
  // TODO Auto-generated destructor stub
}

void
StanceFeetCalculator::Update (const VecFoothold& start_stance,
                              const VecFoothold& steps,
                              const VecSpline& cog_spline)
{
  supp_polygon_container_.Init(start_stance, steps);
  cog_spline_ = cog_spline;
}

StanceFeetCalculator::VecFoothold
StanceFeetCalculator::GetStanceFeetInBase (double t) const
{
  std::vector<xpp::hyq::SupportPolygon> suppport_polygons =
      supp_polygon_container_.CreateSupportPolygonsWith4LS(cog_spline_);

  // legs in contact during each step/spline
  VecFoothold p_stance_legs_i;
  int spline_id = SplineContainer::GetSplineID(t, cog_spline_);
  p_stance_legs_i = suppport_polygons.at(spline_id).footholds_;

  // body position during the step
  xpp::utils::Point2d cog_i = SplineContainer::GetCOGxy(t, cog_spline_);

  return ConvertFeetToBase(p_stance_legs_i, cog_i.p);
}

StanceFeetCalculator::VecFoothold
StanceFeetCalculator::ConvertFeetToBase (VecFoothold endeffectors_i, Vector2d cog_i) const
{
  VecFoothold p_stance_legs_b = endeffectors_i;

  for (uint e=0; e<endeffectors_i.size(); ++e) {
    p_stance_legs_b.at(e).p.x() -= cog_i.x();
    p_stance_legs_b.at(e).p.y() -= cog_i.y();
  }

  return p_stance_legs_b;
}

} /* namespace zmp */
} /* namespace xpp */
