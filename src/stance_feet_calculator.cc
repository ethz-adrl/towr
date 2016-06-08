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
                              const VecSpline& cog_spline,
                              double robot_height)
{
  supp_polygon_container_.Init(start_stance, steps);
  cog_spline_xy_ = cog_spline;
  robot_height_ = robot_height;
}

StanceFeetCalculator::VecFoothold
StanceFeetCalculator::GetStanceFeetInBase (double t) const
{
  std::vector<xpp::hyq::SupportPolygon> suppport_polygons =
      supp_polygon_container_.CreateSupportPolygonsWith4LS(cog_spline_xy_);

  // legs in contact during each step/spline
  VecFoothold p_stance_legs_i;
  int spline_id = SplineContainer::GetSplineID(t, cog_spline_xy_);


  // because last swing support polygon will not restrict landing position of foot
  double T = SplineContainer::GetTotalTime(cog_spline_xy_);

//  std::cout << "T_else: " << T_else << std::endl;
//  double T = SplineContainer::GetDiscretizedGlobalTimes(cog_spline_xy_).back();
//  std::cout << "T: " << T << std::endl;
  if (AreSame(t,T))
    p_stance_legs_i = supp_polygon_container_.GetFinalFootholds();
  else
    p_stance_legs_i = suppport_polygons.at(spline_id).footholds_;

  // body position during the step
  xpp::utils::Point2d cog_xy_i = SplineContainer::GetCOGxy(t, cog_spline_xy_);
  Vector3d cog_i;
  cog_i << cog_xy_i.p.x(),
           cog_xy_i.p.y(),
           robot_height_;

  return ConvertFeetToBase(p_stance_legs_i, cog_i);
}

StanceFeetCalculator::VecFoothold
StanceFeetCalculator::ConvertFeetToBase (const VecFoothold& endeffectors_i,
                                         const Vector3d& cog_i) const
{
  VecFoothold p_stance_legs_b = endeffectors_i;

  for (uint e=0; e<endeffectors_i.size(); ++e) {
    p_stance_legs_b.at(e).p.x() -= cog_i.x();
    p_stance_legs_b.at(e).p.y() -= cog_i.y();
    p_stance_legs_b.at(e).p.z() -= cog_i.z();
  }

  return p_stance_legs_b;
}

bool
StanceFeetCalculator::AreSame (double t1, double t2) const
{
  return std::fabs(t1 - t2) < 1e-4; //seconds
}

} /* namespace zmp */
} /* namespace xpp */
