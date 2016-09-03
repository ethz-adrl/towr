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

StanceFeetCalculator::StanceFeetCalculator (const std::vector<double>& times,
                                            const ComMotion& com_motion,
                                            const SupportPolygonContainer& supp)
{
  Init(times, com_motion, supp);
}

StanceFeetCalculator::~StanceFeetCalculator ()
{
  // TODO Auto-generated destructor stub
}

void
StanceFeetCalculator::Init (const std::vector<double>& times, const ComMotion& com_motion,
                            const SupportPolygonContainer& supp_poly)
{
  com_motion_ = com_motion.clone();
  foothold_container_ = ComSuppPolyPtrU(new SupportPolygonContainer(supp_poly));
  contact_info_vec_ = BuildContactInfoVec(times);

}

void
StanceFeetCalculator::Update (const MotionCoeff& motion_coeff,
                              const PositionVecT& footholds_xy)
{
  com_motion_->SetCoefficients(motion_coeff);
  foothold_container_->SetFootholdsXY(footholds_xy);
}

//StanceFeetCalculator::PositionVecT
//StanceFeetCalculator::CalculateComPostionInWorld () const
//{
//  PositionVecT com_pos;
//  for (double t : times_)
//    com_pos.push_back(com_motion_->GetCom(t).p);
//
//  return com_pos;
//}
//
//StanceFeetCalculator::StanceVecT
//StanceFeetCalculator::GetStanceFootholdsInWorld () const
//{
//  StanceVecT stance_footholds;
//
//  auto supp = foothold_container_->AssignSupportPolygonsToPhases(*com_motion_);
//
//  for (double t : times_) {
//    int phase = com_motion_->GetCurrentPhase(t).id_;
//    stance_footholds.push_back(supp.at(phase).GetFootholds());
//  }
//
//  return stance_footholds;
//}

std::vector<StanceFeetCalculator::ContactInfo>
StanceFeetCalculator::BuildContactInfoVec (const std::vector<double>& times) const
{
  auto supp = foothold_container_->AssignSupportPolygonsToPhases(*com_motion_);

  std::vector<ContactInfo> info;
  for (double t : times) {
    int phase = com_motion_->GetCurrentPhase(t).id_;
    auto stance_feet = supp.at(phase).GetFootholds();

    for (const auto& f : stance_feet)
      info.push_back(ContactInfo(t, f.id, f.leg));
  }

  return info;
}

std::vector<StanceFeetCalculator::ContactInfo>
StanceFeetCalculator::GetContactInfoVec () const
{
  return contact_info_vec_;
}


} /* namespace zmp */
} /* namespace xpp */

