/**
 @file    contact_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/contact_schedule.h>
#include <cassert>

namespace xpp {
namespace opt {

SingleContactMotion::SingleContactMotion ()
{
}

SingleContactMotion::~SingleContactMotion ()
{
}

void
SingleContactMotion::AddPhase (double t_duration)
{
  if (t_phase_end_.empty())
    t_phase_end_.push_back(t_duration);
  else // global time
    t_phase_end_.push_back(t_phase_end_.back() + t_duration);
}

void
SingleContactMotion::SetFirstContactState (bool in_contact)
{
  first_phase_in_contact_ = in_contact;
}

bool
SingleContactMotion::IsInContact (double t_global) const
{
  double eps = 1e-10;   // to ensure that last phases is returned at T

  for (int p=0; p<t_phase_end_.size(); ++p)
    if (t_phase_end_.at(p)+eps >= t_global)
      return GetContact(p);

  assert(false); // t_global longer than trajectory
}

SingleContactMotion::PhaseVec
ContactSchedule::GetPhases (EndeffectorID ee) const
{
  return endeffectors_.At(ee).GetPhases();
}

bool
SingleContactMotion::GetContact (int phase) const
{
   // always alternating
  if (phase%2==0)
    return first_phase_in_contact_;
  else
    return !first_phase_in_contact_;
}

SingleContactMotion::PhaseVec
SingleContactMotion::GetPhases () const
{
  PhaseVec phases;
  double t_prev = 0.0;
  for (int p=0; p<t_phase_end_.size(); ++p) {
    double duration = t_phase_end_.at(p) - t_prev;
    phases.push_back(Phase(GetContact(p),duration));
    t_prev += duration;
  }

  return phases;
}

ContactSchedule::ContactSchedule (const PhaseVec& phases)
    : OptimizationVariables("contact_schedule")
{
  SetPhaseSequence(phases);
}

ContactSchedule::~ContactSchedule ()
{
}

void
ContactSchedule::SetPhaseSequence (const PhaseVec& phases)
{
  int n_ee = phases.front().first.GetCount();
  endeffectors_.SetCount(n_ee);
  Endeffectors<double> durations(n_ee);
  durations.SetAll(0.0);

  SetInitialSwinglegs(phases.front().first);

  for (int i=0; i<phases.size(); ++i) {

    EndeffectorsBool is_swingleg      = phases.at(i).first;
    EndeffectorsBool is_swingleg_next;

    bool last_phase = (i==phases.size()-1);
    if (last_phase)
      is_swingleg_next = is_swingleg.Invert(); // to make sure last phase is always be added
    else
      is_swingleg_next = phases.at(i+1).first;


    for (auto ee : is_swingleg.GetEEsOrdered()) {

      durations.At(ee) += phases.at(i).second;

      // check if next phase is different phase
      bool next_different = is_swingleg.At(ee) != is_swingleg_next.At(ee);

      if (next_different) {
        endeffectors_.At(ee).AddPhase(durations.At(ee));
        durations.At(ee) = 0.0; // reset
      }
    }
  }
}

EndeffectorsBool
ContactSchedule::IsInContact (double t_global) const
{
  EndeffectorsBool contacts(endeffectors_.GetCount());

  for (auto ee :contacts.GetEEsOrdered())
    contacts.At(ee) = endeffectors_.At(ee).IsInContact(t_global);

  return contacts;
}

void
ContactSchedule::SetInitialSwinglegs (const EndeffectorsBool& swinglegs)
{
  for (auto ee :swinglegs.GetEEsOrdered()) {
    bool is_in_contact = !swinglegs.At(ee);
    endeffectors_.At(ee).SetFirstContactState(is_in_contact);
  }
}

int
ContactSchedule::GetContactCount (double t_global) const
{
  int num_contacts = 0;
  for (bool is_contact : IsInContact(t_global).ToImpl())
    num_contacts += is_contact;

  return num_contacts;
}

} /* namespace opt */
} /* namespace xpp */

