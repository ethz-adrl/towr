/*
 * supp_triangle_container.cc
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace hyq {


void SupportPolygonContainer::Init(const VecFoothold& start_stance,
                                   const VecFoothold& footholds,
                                   const MarginValues& margins)
{
  start_stance_  = start_stance;
  footholds_     = footholds;
  margins_       = margins;
  support_polygons_ = CreateSupportPolygons(footholds);

  initialized_ = true;
}


void SupportPolygonContainer::Init(const VecFoothold& start_stance,
                                   const VecLegID& step_sequence,
                                   const MarginValues& margins)
{
  // initial all footholds with the correct leg, but x=y=z=0.0
  std::vector<xpp::hyq::Foothold> zero_footholds;
  for (uint i=0; i<step_sequence.size(); ++i) {
    xpp::hyq::Foothold f; // sets x=y=z=0.0
    f.leg   = step_sequence.at(i);
    footholds_.push_back(f);
  }

  start_stance_  = start_stance;
  margins_       = margins;
  support_polygons_ = CreateSupportPolygons(footholds_);

  initialized_ = true;
}


void SupportPolygonContainer::SetFootholdsXY(int idx, double x, double y)
{
  footholds_.at(idx).p.x() = x;
  footholds_.at(idx).p.y() = y;

  support_polygons_ = CreateSupportPolygons(footholds_); //update support polygons as well
};


SupportPolygon SupportPolygonContainer::GetStancePolygon(const VecFoothold& footholds) const
{
  CheckIfInitialized();
  return SupportPolygon(margins_, footholds);
}


SupportPolygonContainer::VecFoothold
SupportPolygonContainer::GetStanceDuring(int step) const
{
  // last four leg support phase has all legs on ground
  if (step == GetNumberOfSteps())
    return GetFinalFootholds();
  else
    return support_polygons_.at(step).footholds_;
}


SupportPolygonContainer::VecFoothold
SupportPolygonContainer::GetStanceAfter(int n_steps) const
{
  CheckIfInitialized();

  VecFoothold combined = start_stance_;
  combined.insert(combined.end(), footholds_.begin(), footholds_.begin()+n_steps);


  // get the last step of each foot
  VecFoothold last_stance;
  Foothold f;
  for (LegID l : LegIDArray)
    if(Foothold::GetLastFoothold(l,combined, f))
      last_stance.push_back(f);

  return last_stance;
}


SupportPolygon SupportPolygonContainer::GetStartPolygon() const
{
  return GetStancePolygon(start_stance_);
}


SupportPolygon SupportPolygonContainer::GetFinalPolygon() const
{
  return GetStancePolygon(GetFinalFootholds());
}

SupportPolygonContainer::VecFoothold
SupportPolygonContainer::GetFinalFootholds() const
{
  return GetStanceAfter(footholds_.size());
}



SupportPolygonContainer::VecSupportPolygon
SupportPolygonContainer::CreateSupportPolygons(const VecFoothold& footholds) const
{
  std::vector<SupportPolygon> supp;
  VecFoothold curr_stance = start_stance_;

  for (const Foothold& f : footholds) {

    // extract the 3 non-swinglegs from stance
    VecFoothold legs_in_contact;
    for (const Foothold& f_curr : curr_stance)
      if(f_curr.leg != f.leg)
        legs_in_contact.push_back(f_curr);

    supp.push_back(SupportPolygon(margins_, legs_in_contact));

    // update current stance after step
    Foothold::UpdateFoohold(f, curr_stance);
  }

  return supp;
}


Eigen::Vector2d
SupportPolygonContainer::GetCenterOfFinalStance() const
{
  CheckIfInitialized();

  VecFoothold last_stance = GetFinalFootholds();

  // calculate average x-y-postion of last stance
  Eigen::Vector2d end_cog = Eigen::Vector2d::Zero();

  for (const Foothold& f : last_stance)
    end_cog += f.p.segment<2>(xpp::utils::X);

  return end_cog/_LEGS_COUNT;
}


bool SupportPolygonContainer::Insert4LSPhase(LegID prev, LegID next)
{
  using namespace xpp::hyq;
  // check for switching between disjoint support triangles.
  // the direction the robot is moving between triangles does not matter.
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;
  std::swap(prev, next);
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;

  return false;
}


void
SupportPolygonContainer::CheckIfInitialized() const
{
  if (!initialized_) {
    throw std::runtime_error("SuppTriangleContainer not initialized. Call Init() first");
  }
}


} /* namespace hyq */
} /* namespace xpp */
