/**
 @file    motion_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the MotionStructure class.
 */

#include <xpp/opt/motion_structure.h>
#include <algorithm>

namespace xpp {
namespace opt {

MotionStructure::MotionStructure ()
{
}

MotionStructure::~MotionStructure ()
{
  // TODO Auto-generated destructor stub
}

// mpc clean this up, messy code
void
MotionStructure::Init (const StartStance& start_stance,
                       const EEIDVec& contact_ids,
                       double t_swing, double t_first_phase,
                       bool insert_initial_stance,
                       bool insert_final_stance,
                       double dt)
{
  // doesn't have to be 4 legs on the ground
  if (insert_initial_stance) {
    MotionPhase initial_stance_phase;
    initial_stance_phase.duration_ = t_first_phase;
    initial_stance_phase.fixed_contacts_ = start_stance;
    phases_.push_back(initial_stance_phase);
  }

  std::vector<ContactBase> all_free_contacts;
  int j = 0;
  for (auto leg : contact_ids) {
    all_free_contacts.push_back(ContactBase(j++, leg));
  }


  MotionPhase prev_phase;
  prev_phase.fixed_contacts_ = start_stance;
  int n_swinglegs = 1; // per phase
  int n_phases = std::floor(contact_ids.size()/n_swinglegs);

  // the steps
  for (uint i=0; i<n_phases; ++i) {

    MotionPhase phase;
    phase.fixed_contacts_ = prev_phase.fixed_contacts_;
    phase.free_contacts_  = prev_phase.free_contacts_;
    // add newly made contact of previous swing
    phase.free_contacts_.insert(phase.free_contacts_.end(), prev_phase.swing_goal_contacts_.begin(),
                                                            prev_phase.swing_goal_contacts_.end());


    std::vector<ContactBase> swinglegs;
    for (int l=0; l<n_swinglegs; ++l) {
      swinglegs.push_back(all_free_contacts.at(i*n_swinglegs+l));
    }

    for (const auto& sl : swinglegs) {

      // this contact is swinging during this phase
      // amazing idea! (pats back)
//      Contact goal_contact(i, sl_contact);
      phase.swing_goal_contacts_.push_back(sl);


      // remove current swinglegs from list of active contacts
      auto it_fixed = std::find_if(phase.fixed_contacts_.begin(), phase.fixed_contacts_.end(),
                                   [&](const Contact& f) {return f.ee == sl.ee;});

      if (it_fixed != phase.fixed_contacts_.end()) // step found in initial stance
        phase.fixed_contacts_.erase(it_fixed);     // remove contact, because now swinging leg


      // remove current swingles from last free contacts
      auto it_free = std::find_if(phase.free_contacts_.begin(), phase.free_contacts_.end(),
                                  [&](const ContactBase& c) {return c.ee == sl.ee;});

      if (it_free != phase.free_contacts_.end()) // step found in current stance
        phase.free_contacts_.erase(it_free);     // remove contact, because now swinging leg

    }


    phase.duration_ = phases_.empty() ? t_first_phase : t_swing;
    phases_.push_back(phase);

    prev_phase = phase;
  }

  // the final stance
  if (insert_final_stance) {
    MotionPhase phase;
    phase.fixed_contacts_    = prev_phase.fixed_contacts_;
    phase.free_contacts_     = prev_phase.free_contacts_;
    phase.free_contacts_.insert(phase.free_contacts_.end(), prev_phase.swing_goal_contacts_.begin(), prev_phase.swing_goal_contacts_.end());
    phase.duration_ = 0.5;
    phases_.push_back(phase);
  }


  for (auto p : phases_) {
    std::cout << p << std::endl << std::endl;;
  }


  start_stance_ = start_stance;
  contact_ids_ = contact_ids;
  dt_ = dt;
  cache_needs_updating_ = true;
}

MotionPhase
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


MotionStructure::PhaseStampedVec
MotionStructure::GetPhaseStampedVec () const
{
  if (cache_needs_updating_) {
    cached_motion_vector_ = CalcPhaseStampedVec();
    cache_needs_updating_ = false;
  }

  return cached_motion_vector_;
}

MotionStructure::PhaseStampedVec
MotionStructure::CalcPhaseStampedVec () const
{
  PhaseStampedVec info;

  double t_global = 0;
  for (auto phase : phases_) {

    int nodes_in_phase = std::floor(phase.duration_/dt_);

    // zmp_ make sure they are always evenly spaced! dt is the same also between nodes
    for (int k=0; k<nodes_in_phase; ++k ) {
      MotionNode contact_info;
      contact_info = phase;
      contact_info.time_  = t_global+k*dt_;
      info.push_back(contact_info);
    }

    t_global += phase.duration_;
  }

  return info;
}

int
MotionStructure::GetTotalNumberOfFreeNodeContacts () const
{
  auto contact_info_vec = GetPhaseStampedVec();

  int i = 0;
  for (auto node : contact_info_vec)
    i += node.free_contacts_.size();

  return i;
}

int
MotionStructure::GetTotalNumberOfNodeContacts () const
{
  auto contact_info_vec = GetPhaseStampedVec();

  int i = 0;
  for (auto node : contact_info_vec) {
    i += node.free_contacts_.size();
    i += node.fixed_contacts_.size();
  }

  return i;
}

MotionStructure::PhaseVec
MotionStructure::GetPhases () const
{
  return phases_;
}

} /* namespace opt */
} /* namespace xpp */

