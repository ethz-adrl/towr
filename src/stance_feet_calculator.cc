/**
 @file    stance_feet_calculator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/zmp/stance_feet_calculator.h>

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/com_motion.h>

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
StanceFeetCalculator::Init (const std::vector<double>& times, const ComMotion& com_motion,
                            const SupportPolygonContainer& supp_poly)
{
  times_ = times;
  com_motion_ = com_motion.clone();
  supp_polygon_container_ = ComSuppPolyPtrU(new SupportPolygonContainer(supp_poly));
}

void
StanceFeetCalculator::Update (const MotionCoeff& motion_coeff,
                              const PositionVecT& footholds_xy)
{
  com_motion_->SetCoefficients(motion_coeff);
  supp_polygon_container_->SetFootholdsXY(footholds_xy);
}

StanceFeetCalculator::PositionVecT
StanceFeetCalculator::CalculateComPostionInWorld () const
{
  PositionVecT com_pos;
  for (double t : times_)
    com_pos.push_back(com_motion_->GetCom(t).p);

  return com_pos;
}

StanceFeetCalculator::StanceVecT
StanceFeetCalculator::GetStanceFootholdsInWorld () const
{
  StanceVecT stance_footholds;

  auto supp = supp_polygon_container_->AssignSupportPolygonsToPhases(*com_motion_);

  for (double t : times_) {
    int phase = com_motion_->GetCurrentPhase(t).id_;
    stance_footholds.push_back(supp.at(phase).GetFootholds());
  }

  return stance_footholds;
}

//void
//StanceFeetCalculator::Update (const StanceFootholds& start_stance,
//                              const StanceFootholds& steps,
//                              const ComSplinePtr& cog_spline,
//                              double robot_height)
//{
//  supp_polygon_container_.Init(start_stance, steps);
//  com_motion_ = cog_spline;
//  robot_height_ = robot_height;
//}
//
//StanceFeetCalculator::StanceFootholds
//StanceFeetCalculator::GetStanceFeetInBase (double t) const
//{
//  std::vector<xpp::hyq::SupportPolygon> suppport_polygons =
//      supp_polygon_container_.AssignSupportPolygonsToPhases(*com_motion_);
//
//  // legs in contact during each step/spline
//  StanceFootholds p_stance_legs_i;
//  int phase_id = com_motion_->GetCurrentPhase(t).id_;
//
//
//  // because last swing support polygon will not restrict landing position of foot
//  double T = com_motion_->GetTotalTime();
//
//  if (AreSame(t,T))
//    p_stance_legs_i = supp_polygon_container_.GetFinalFootholds();
//  else
//    p_stance_legs_i = suppport_polygons.at(phase_id).GetFootholds();
//
//  // body position during the step
//  xpp::utils::Point2d cog_xy_i = com_motion_->GetCom(t);
//  PosXYZ cog_i;
//  cog_i << cog_xy_i.p.x(),
//           cog_xy_i.p.y(),
//           robot_height_;
//
//  return ConvertFeetToBase(p_stance_legs_i, cog_i);
//}
//
//StanceFeetCalculator::StanceFootholds
//StanceFeetCalculator::ConvertFeetToBase (const StanceFootholds& endeffectors_i,
//                                         const PosXYZ& cog_i) const
//{
//  StanceFootholds p_stance_legs_b = endeffectors_i;
//
//  for (uint e=0; e<endeffectors_i.size(); ++e) {
//    p_stance_legs_b.at(e).p.x() -= cog_i.x();
//    p_stance_legs_b.at(e).p.y() -= cog_i.y();
//    p_stance_legs_b.at(e).p.z() -= cog_i.z();
//  }
//
//  return p_stance_legs_b;
//}
//
//bool
//StanceFeetCalculator::AreSame (double t1, double t2) const
//{
//  return std::fabs(t1 - t2) < 1e-4; //seconds
//}

} /* namespace zmp */
} /* namespace xpp */
