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
MotionStructure::Init (const EEIDVec& all_robot_ee,
                       const StartStance& ee_pos,
                       const AllPhaseSwingLegs& phase_swinglegs,
                       double percent_first_phase,
                       double dt)
{
  int contact_id = 0;
  MotionPhase prev_phase;

  for (EEID ee : ee_pos.GetEEsOrdered()) {
    Contact c;
    c.ee = ee;
    c.p = ee_pos.At(ee);
    prev_phase.contacts_fixed_.push_back(c);
  }

  for (uint i=0; i<phase_swinglegs.size(); ++i) {

    MotionPhase phase = prev_phase;

    EEIDVec stancelegs;
    for (const auto& ee : all_robot_ee) {
      auto& v = phase_swinglegs.at(i).first; //alias
      auto it = std::find_if(v.begin(), v.end(), [&](const EEID& leg) {return leg == ee;});
      if (it == v.end()) // endeffector not in swinglegs
        stancelegs.push_back(ee);
    }

    // stance to swing
    for (const auto& ee : stancelegs)
      if (prev_phase.IsInSwinglegs(ee))
          phase.ShiftSwingToStance(ee);

    // stance to swing
    for (const auto& ee : phase_swinglegs.at(i).first) {
      if (!prev_phase.IsInSwinglegs(ee)) {

        ContactBase sl(contact_id++, ee);
        phase.swinglegs_.push_back(sl);
        phase.RemoveContact(ee); // because phase is now swingleg
      }
    }


    // first phase can have shorter duration
    phase.duration_ = phase_swinglegs.at(i).second;
    phases_.push_back(phase);

    prev_phase = phase;
  }

  std::cout << "Motion Phases:\n";
  for (auto p : phases_) {
    std::cout << p << std::endl << std::endl;;
  }

  phase_swing_ee_ = phase_swinglegs;
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

std::vector<MotionStructure::EEID>
MotionStructure::GetContactIds () const
{
  std::vector<EEID> all_ee;
  for (auto phase_contact : phase_swing_ee_)
    for(auto ee : phase_contact.first)
      all_ee.push_back(ee);

  return all_ee;
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
    i += node.contacts_opt_.size();

  return i;
}

int
MotionStructure::GetTotalNumberOfNodeContacts () const
{
  auto contact_info_vec = GetPhaseStampedVec();

  int i = 0;
  for (auto node : contact_info_vec) {
    i += node.contacts_opt_.size();
    i += node.contacts_fixed_.size();
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

