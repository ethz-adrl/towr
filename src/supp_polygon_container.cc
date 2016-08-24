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
  SetStartStance(start_stance);
  SetFootholds(footholds);
  margins_       = margins;
  support_polygons_ = CreateSupportPolygons(footholds);

  initialized_ = true;
}

void SupportPolygonContainer::Init(const VecFoothold& start_stance,
                                   const VecLegID& step_sequence,
                                   const MarginValues& margins)
{
  // initial all footholds with the correct leg, but x=y=z=0.0
  for (uint i=0; i<step_sequence.size(); ++i) {
    xpp::hyq::Foothold f; // sets x=y=z=0.0
    f.leg   = step_sequence.at(i);
    footholds_.push_back(f);
  }

  SetStartStance(start_stance);
  SetFootholds(footholds_);
  margins_       = margins;
  support_polygons_ = CreateSupportPolygons(footholds_);

  initialized_ = true;
}

void
SupportPolygonContainer::SetFootholdsXY(const StdVecEigen2d& footholds_xy)
{
  assert(footholds_xy.size() == footholds_.size());
  for (uint i=0; i<footholds_xy.size(); ++i)
    footholds_.at(i).SetXy(footholds_xy.at(i).x(), footholds_xy.at(i).y());

  support_polygons_ = CreateSupportPolygons(footholds_); //update support polygons as well
}

Eigen::VectorXd
SupportPolygonContainer::GetFootholdsInitializedToStart() const
{
  StdVecEigen2d footholds_xy(footholds_.size());

  for (uint step=0; step<footholds_.size(); ++step) {
    xpp::hyq::LegID leg = footholds_.at(step).leg;
    footholds_xy.at(step) = GetStartFoothold(leg).GetXy();
  }

  return utils::ConvertStdToEig(footholds_xy);
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
  VecSupportPolygon supp = AssignSupportPolygonsToSplines(splines);

  std::vector<SupportPolygon::VecSuppLine> supp_lines;
  for (uint s=0; s<splines.size(); ++s) {
    supp_lines.push_back(supp.at(s).CalcLines());
  }

  return supp_lines;
}

SupportPolygonContainer::VecSupportPolygon
SupportPolygonContainer::AssignSupportPolygonsToSplines(const VecZmpSpline& splines) const
{
  using namespace xpp::zmp;

  VecSupportPolygon supp_steps = GetSupportPolygons();

  VecSupportPolygon supp;
  for (const auto& s : splines) {
    SupportPolygon curr_supp;
    PhaseInfo phase = s.phase_;


    int prev_step = phase.n_completed_steps_-1;
    switch (phase.type_) {
      case kStepPhase: {
        curr_supp = supp_steps.at(prev_step+1);
        break;
      }
      case kStancePhase: {
        if (prev_step == -1) // first spline
          curr_supp = GetStartPolygon();
        else if (prev_step == GetNumberOfSteps()-1)
          curr_supp = GetFinalPolygon();
        else // for intermediate splines
          curr_supp = SupportPolygon::CombineSupportPolygons(supp_steps.at(prev_step), supp_steps.at(prev_step+1));
      }
    }


    supp.push_back(curr_supp);
  }

  return supp;
}

SupportPolygonContainer::VecSupportPolygon
SupportPolygonContainer::AssignSupportPolygonsToPhases(const ComMotion& com_motion) const
{
  using namespace xpp::zmp;

  VecSupportPolygon supp_steps = GetSupportPolygons();

  VecSupportPolygon supp;
  for (const auto& phase : com_motion.GetPhases()) {
    SupportPolygon curr_supp;


    int prev_step = phase.n_completed_steps_-1;
    switch (phase.type_) {
      case kStepPhase: {
        curr_supp = supp_steps.at(prev_step+1);
        break;
      }
      case kStancePhase: {
        if (prev_step == -1) // first spline
          curr_supp = GetStartPolygon();
        else if (prev_step == GetNumberOfSteps()-1)
          curr_supp = GetFinalPolygon();
        else // for intermediate splines
          curr_supp = SupportPolygon::CombineSupportPolygons(supp_steps.at(prev_step), supp_steps.at(prev_step+1));
      }
    }





    supp.push_back(curr_supp);
  }

  return supp;
}


void
SupportPolygonContainer::CheckIfInitialized() const
{
  if (!initialized_) {
    throw std::runtime_error("SuppTriangleContainer not initialized. Call Init() first");
  }
}

void
SupportPolygonContainer::SetStartStance (const VecFoothold& start_stance)
{
  start_stance_ = start_stance;
  for (auto& f : start_stance_)
    f.fixed_by_start_stance = true;
}

void
SupportPolygonContainer::SetFootholds (const VecFoothold& footholds)
{
  footholds_ = footholds;
  for (auto& f : footholds_)
    f.fixed_by_start_stance = false;
}

} /* namespace hyq */
} /* namespace xpp */

