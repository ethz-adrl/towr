/**
 @file    support_polygon_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Defines the SupportPolygonContainer class
 */

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/motion_structure.h>
#include <xpp/zmp/phase_info.h>
#include <xpp/utils/cartesian_declarations.h>

namespace xpp {
namespace hyq {

void SupportPolygonContainer::Init(const VecFoothold& start_stance,
                                   const VecFoothold& footholds,
                                   const MarginValues& margins)
{
  start_stance_ = start_stance;
  footholds_ = footholds;
  margins_       = margins;

  ModifyFootholds(start_stance_, [](Foothold& f, int i) {f.id = Foothold::kFixedByStart;} );
  ModifyFootholds(footholds_,    [](Foothold& f, int i) {f.id = i;} );

  support_polygons_ = CreateSupportPolygons(footholds_);
}

void SupportPolygonContainer::Init(const VecFoothold& start_stance,
                                   const VecLegID& step_sequence,
                                   const MarginValues& margins)
{
  VecFoothold footholds;
  // initial all footholds with the correct leg, but x=y=z=0.0
  for (uint i=0; i<step_sequence.size(); ++i) {
    xpp::hyq::Foothold f; // sets x=y=z=0.0
    f.leg   = step_sequence.at(i);
    footholds.push_back(f);
  }

  Init(start_stance, footholds, margins);
}

void
SupportPolygonContainer::Init (const VecLegID& start_legs,
                               const VecLegID& step_sequence,
                               const MarginValues& margins)
{
  VecFoothold start_stance;
  for (uint i=0; i<start_legs.size(); ++i) {
    xpp::hyq::Foothold f; // sets x=y=z=0.0
    f.leg   = start_legs.at(i);
    start_stance.push_back(f);
  }

  Init(start_stance, step_sequence, margins);
}

void
SupportPolygonContainer::SetFootholdsXY(const StdVecEigen2d& footholds_xy)
{
  assert(footholds_xy.size() == footholds_.size());
  Foothold::SetXy(footholds_xy, footholds_);
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
  return SupportPolygon(footholds, margins_);
}

SupportPolygonContainer::VecFoothold
SupportPolygonContainer::GetStanceDuring(int step) const
{
  // last four leg support phase has all legs on ground
  if (step == GetNumberOfSteps())
    return GetFinalFootholds();
  else
    return support_polygons_.at(step).GetFootholds();
}

SupportPolygonContainer::VecFoothold
SupportPolygonContainer::GetStanceAfter(int n_steps) const
{
  VecFoothold combined = start_stance_;
  combined.insert(combined.end(), footholds_.begin(), footholds_.begin()+n_steps);

  // get the last step of each foot
  VecFoothold last_stance;
  for (LegID l : LegIDArray) {

    if(std::find_if(combined.begin(), combined.end(),
                 [&l](const Foothold& f) {return f.leg == l;}) != combined.end())
    {
      last_stance.push_back(Foothold::GetLastFoothold(l,combined));
    }

  }
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
  VecFoothold last_stance = GetFinalFootholds();

  // calculate average x-y-postion of last stance
  Eigen::Vector2d end_cog = Eigen::Vector2d::Zero();

  for (const Foothold& f : last_stance)
    end_cog += f.p.topRows<utils::kDim2d>();

  return end_cog/_LEGS_COUNT;
}

SupportPolygonContainer::VecSupportPolygon
SupportPolygonContainer::AssignSupportPolygonsToPhases(const PhaseInfoVec& phases) const
{
  VecSupportPolygon supp;

  for (const auto& phase : phases) {

    VecFoothold contacts;
    for (auto c : phase.free_contacts_)
      contacts.push_back(footholds_.at(c.id));

    for (auto f : phase.fixed_contacts_)
      contacts.push_back(f);

    supp.push_back(SupportPolygon(contacts, margins_));
  }

  return supp;
}

int
SupportPolygonContainer::GetTotalFreeCoeff () const
{
  return GetNumberOfSteps()*utils::kDim2d;
}

int
SupportPolygonContainer::Index (int id, Coords dim)
{
  assert(id != Foothold::kFixedByStart); // they can't be optimized, have no index
  return id*xpp::utils::kDim2d + dim;
}

void
SupportPolygonContainer::ModifyFootholds (VecFoothold& footholds,
                                          std::function<void (Foothold&, int)> fun) const
{
  int i = 0;

  for (auto& f : footholds)
    fun(f, i++);
}

bool
SupportPolygonContainer::DisJointSupportPolygons(LegID prev, LegID next)
{
  // check for switching between disjoint support triangles.
  // the direction the robot is moving between triangles does not matter.
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;
  std::swap(prev, next);
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;

  return false;
}

} /* namespace hyq */
} /* namespace xpp */
