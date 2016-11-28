/**
 @file    motion_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the MotionStructure class.
 */

#include <xpp/opt/motion_structure.h>

#include <sys/types.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>

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

// mpc clean this up, messy code
void
MotionStructure::Init (const StartStance& start_stance,
                       const LegIDVec& step_legs,
                       double t_swing, double t_first_phase,
                       bool insert_initial_stance,
                       bool insert_final_stance)
{
  int id = -1;


  if (insert_initial_stance) {
    PhaseInfo initial_stance_phase;
    initial_stance_phase.id_ = ++id;
    initial_stance_phase.duration_ = t_first_phase;
    initial_stance_phase.fixed_contacts_ = start_stance;
    initial_stance_phase.n_completed_steps_ = 0;
    phases_.push_back(initial_stance_phase);
  }


  PhaseInfo prev_phase;
  prev_phase.fixed_contacts_ = start_stance;
  // the steps
  for (uint i=0; i<step_legs.size(); ++i) {

    PhaseInfo phase;
    phase.free_contacts_  = prev_phase.free_contacts_;
    phase.fixed_contacts_ = prev_phase.fixed_contacts_;

    // remove current swingleg from list of active contacts
    auto it_fixed = std::find_if(phase.fixed_contacts_.begin(), phase.fixed_contacts_.end(),
                           [&](const Foothold& f) {return f.leg == step_legs.at(i);});

    if (it_fixed != phase.fixed_contacts_.end()) // step found in initial stance
      phase.fixed_contacts_.erase(it_fixed);     // remove contact, because now swinging leg


    // remove current swingleg from last free contacts
    auto it_free = std::find_if(phase.free_contacts_.begin(), phase.free_contacts_.end(),
                           [&](const Contact& c) {return c.ee == static_cast<EndeffectorID>(step_legs.at(i));});

    if (it_free != phase.free_contacts_.end()) // step found in current stance
      phase.free_contacts_.erase(it_free);     // remove contact, because now swinging leg



    // add newly made contact of previous swing
    if (i > 0)
      phase.free_contacts_.push_back(Contact(i-1, static_cast<EndeffectorID>(step_legs.at(i-1))));

    phase.id_ = ++id;
    phase.duration_ = phase.id_==0? t_first_phase : t_swing;
    phase.n_completed_steps_ = i;
    phases_.push_back(phase);

    prev_phase = phase;
  }

  // the final stance
  if (insert_final_stance) {
    PhaseInfo phase = phases_.back();

    if (phase.IsStep()) {
      int last_contact_id = step_legs.size()-1;
      phase.free_contacts_.push_back(Contact(last_contact_id, static_cast<EndeffectorID>(step_legs.back())));
      phase.n_completed_steps_++;
    }

    phase.id_ = ++id;
    phase.duration_ = 1.05;
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


PhaseStampedVec
MotionStructure::GetPhaseStampedVec () const
{
  if (cache_needs_updating_) {
    cached_motion_vector_ = CalcPhaseStampedVec();
    cache_needs_updating_ = false;
  }

  return cached_motion_vector_;
}

PhaseStampedVec
MotionStructure::CalcPhaseStampedVec () const
{
  PhaseStampedVec info;

  double t_global = 0;
  for (auto phase : phases_) {

    int nodes_in_phase = std::floor(phase.duration_/dt_);

    for (int k=0; k<nodes_in_phase; ++k ) {

      PhaseInfoStamped contact_info;
      contact_info.phase_ = phase;
      contact_info.time_  = t_global+k*dt_;

      info.push_back(contact_info);
    }

    t_global += phase.duration_;
  }

  return info;
}

int
MotionStructure::GetTotalNumberOfDiscreteContacts () const
{
  auto contact_info_vec = GetPhaseStampedVec();

  int i = 0;
  for (auto node : contact_info_vec)
    i += node.phase_.free_contacts_.size();

  return i;
}

PhaseVec
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

