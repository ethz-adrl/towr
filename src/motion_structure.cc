/**
 @file    motion_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the MotionStructure class.
 */

#include "../include/xpp/opt/motion_structure.h"

#include <xpp/hyq/support_polygon_container.h>
#include <algorithm>
#include "../include/xpp/opt/phase_info.h"

namespace xpp {
namespace opt {

using Foothold = xpp::hyq::Foothold;

MotionStructure::MotionStructure ()
{
  dt_ = 0.2; //standart discretization
}

MotionStructure::~MotionStructure ()
{
  // TODO Auto-generated destructor stub
}

void
MotionStructure::Init (const StartStance& start_stance,
                       const LegIDVec& step_legs,
                       double t_swing, double t_stance,
                       bool insert_initial_stance,
                       bool insert_final_stance)
{

  PhaseInfo initial_stance_phase;
  initial_stance_phase.id_ = 0;
  initial_stance_phase.duration_ = t_stance;
  initial_stance_phase.fixed_contacts_ = start_stance;
  initial_stance_phase.n_completed_steps_ = 0;

  if (insert_initial_stance) {
    phases_.push_back(initial_stance_phase);
  }

  // the steps
  for (uint i=0; i<step_legs.size(); ++i) {

    PhaseInfo phase = i==0 ? initial_stance_phase : phases_.back();

    // remove current swingleg from list of active contacts
    auto it_fixed = std::find_if(phase.fixed_contacts_.begin(), phase.fixed_contacts_.end(),
                           [&](const Foothold& f) {return f.leg == step_legs.at(i);});

    if (it_fixed != phase.fixed_contacts_.end()) // step found in initial stance
      phase.fixed_contacts_.erase(it_fixed);     // remove contact, because now swinging leg



    // remove current swingleg from last free contacts
    auto it_free = std::find_if(phase.free_contacts_.begin(), phase.free_contacts_.end(),
                           [&](const Contact& c) {return c.ee == static_cast<EndeffectorID>(step_legs.at(i));});

    if (it_free != phase.free_contacts_.end()) // step found in initial stance
      phase.free_contacts_.erase(it_free);     // remove contact, because now swinging leg



    // add newly made contact of previous swing
    if (i > 0)
      phase.free_contacts_.push_back(Contact(i-1, static_cast<EndeffectorID>(step_legs.at(i-1))));

    phase.id_++;
    phase.duration_ = t_swing;
    phase.n_completed_steps_ = i;
    phases_.push_back(phase);
  }

  // the final stance
  if (insert_final_stance) {
    PhaseInfo phase = phases_.back();

    if (phase.IsStep()) {
      int last_contact_id = step_legs.size()-1;
      phase.free_contacts_.push_back(Contact(last_contact_id, static_cast<EndeffectorID>(step_legs.back())));
      phase.n_completed_steps_++;
    }

    phase.id_++;
    phase.duration_ = 0.05;
    phases_.push_back(phase);
  }


  start_stance_ = start_stance;
  steps_ = step_legs;
  cache_needs_updating_ = true;
}

PhaseInfo
MotionStructure::GetCurrentPhase (double t_global) const
{
  double t = 0;
  for (const auto& phase: phases_) {
    t += phase.duration_;

    if (t >= t_global) // at junctions, returns previous phase (=)
      return phase;
  }
  assert(false); // this should never be reached
}

double
MotionStructure::GetTotalTime() const
{
  double T = 0.0;
  for (const auto& phase: phases_)
    T += phase.duration_;
  return T;
}


MotionStructure::MotionInfoVec
MotionStructure::GetContactInfoVec () const
{
  if (cache_needs_updating_) {
    cached_motion_vector_ = CalcContactInfoVec();
    cache_needs_updating_ = false;
  }

  return cached_motion_vector_;
}

MotionStructure::MotionInfoVec
MotionStructure::CalcContactInfoVec () const
{
  xpp::hyq::SupportPolygonContainer foothold_container;
  foothold_container.Init(start_stance_, steps_);
  auto supp = foothold_container.AssignSupportPolygonsToPhases(phases_);

  MotionInfoVec info;

  double t_global = 0;
  for (auto phase : phases_) {

    auto stance_feet = supp.at(phase.id_).GetFootholds();

    int nodes_in_phase = std::floor(phase.duration_/dt_);

    for (int k=0; k<nodes_in_phase; ++k ) {

      MotionInfo contact_info;
      contact_info.time_ = t_global+k*dt_;

      for (const auto& f : stance_feet)
        contact_info.phase_.free_contacts_.push_back(Contact(f.id, static_cast<EndeffectorID>(f.leg)));

      info.push_back(contact_info);
    }

    t_global += phase.duration_;
  }

  // even though the last footstep doesn't create a support polygon, still include
  // this last time instance with contact configuration
  MotionInfo final_contacts;
  final_contacts.time_ = t_global;
  for (const auto& f : foothold_container.GetFinalFootholds())
    final_contacts.phase_.free_contacts_.push_back(Contact(f.id, static_cast<EndeffectorID>(f.leg)));

  info.push_back(final_contacts);

  return info;
}

int
MotionStructure::GetTotalNumberOfDiscreteContacts () const
{
  auto contact_info_vec = GetContactInfoVec();

  int i = 0;
  for (auto node : contact_info_vec)
    i += node.phase_.free_contacts_.size();

  return i;
}

MotionStructure::PhaseVec
MotionStructure::GetPhases () const
{
  return phases_;
}

void
MotionStructure::SetDisretization (double dt)
{
  dt_ = dt;
  cache_needs_updating_ = true;
}

} /* namespace zmp */
} /* namespace xpp */

