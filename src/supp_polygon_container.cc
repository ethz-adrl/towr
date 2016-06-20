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
  footholds_.at(idx).SetXy(x,y);
  support_polygons_ = CreateSupportPolygons(footholds_); //update support polygons as well
}

SupportPolygonContainer::StdVecEigen2d
SupportPolygonContainer::GetFootholdsInitializedToStart() const
{
  StdVecEigen2d footholds_xy(footholds_.size());

  for (uint step=0; step<footholds_.size(); ++step) {
    xpp::hyq::LegID leg = footholds_.at(step).leg;
    footholds_xy.at(step) = GetStartFoothold(leg).GetXy();
  }

  return footholds_xy;
}

SupportPolygon SupportPolygonContainer::GetStancePolygon(const VecFoothold& footholds) const
{
  CheckIfInitialized();
  return SupportPolygon(footholds, margins_);
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
    last_stance.push_back(Foothold::GetLastFoothold(l,combined));

  return last_stance;
}

Foothold SupportPolygonContainer::GetStartFoothold(LegID leg) const
{
  return Foothold::GetLastFoothold(leg, start_stance_);
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

    supp.push_back(SupportPolygon(legs_in_contact, margins_));

    // update current stance after step
    Foothold::UpdateFoothold(f, curr_stance);
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

SupportPolygonContainer::VecVecSuppLine
SupportPolygonContainer::GetActiveConstraintsForEachStep(const VecZmpSpline& splines) const
{
  VecSupportPolygon supp = CreateSupportPolygonsWith4LS(splines);

  std::vector<SupportPolygon::VecSuppLine> supp_lines;
  for (uint s=0; s<splines.size(); ++s) {
    supp_lines.push_back(supp.at(s).CalcLines());
  }

  return supp_lines;
}

SupportPolygonContainer::VecSupportPolygon
SupportPolygonContainer::CreateSupportPolygonsWith4LS(const VecZmpSpline& splines) const
{
  using namespace xpp::zmp;

  // support polygons during step and in four leg support phases
  VecSupportPolygon supp_no_4l = GetSupportPolygons();

  VecSupportPolygon supp;
  for (const ZmpSpline& s : splines)
  {
    SupportPolygon curr_supp;
    switch (s.GetType()) {
      case Initial4lsSpline: {
        curr_supp = GetStartPolygon();
        break;
      }
      case StepSpline: {
        curr_supp = supp_no_4l.at(s.GetCurrStep());
        break;
      }
      case Intermediate4lsSpline: {
        int next = s.GetNextPlannedStep();
        curr_supp = SupportPolygon::CombineSupportPolygons(supp_no_4l.at(next),
                                                           supp_no_4l.at(next-1));
        break;
      }
      case Final4lsSpline: {
        curr_supp = GetFinalPolygon();
        break;
      }
      default:
        std::cerr << "CreateSuppPolygonswith4ls: Could not categorize spline";
        break;
    }

    supp.push_back(curr_supp);
  }

  return supp;
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
