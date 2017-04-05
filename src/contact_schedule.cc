/**
 @file    contact_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#include "../include/xpp/opt/contact_schedule.h"

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
  bool contact = first_phase_in_contact_;
  double eps = 1e-10;   // to ensure that last phases is returned at T
  for (double t : t_phase_end_) {

    if (t+eps >= t_global)
      return contact;

    contact = !contact; // stance and swing phase alternating
  }

  assert(false); // t_global longer than trajectory
}

ContactSchedule::ContactSchedule () : Parametrization("Contact Motion")
{
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

  for (int i=0; i<phases.size()-1; ++i) {

    EndeffectorsBool is_swingleg      = phases.at(i).first;
    EndeffectorsBool is_swingleg_next = phases.at(i+1).first;
    double phase_duration             = phases.at(i).second;

    for (auto ee : is_swingleg.GetEEsOrdered()) {

      durations.At(ee) += phase_duration;

      // check if next phase is different phase
      bool next_different = is_swingleg.At(ee) != is_swingleg_next.At(ee);

      if (next_different) {

        if (!is_swingleg.At(ee)) { // stance leg to swing
          endeffectors_.At(ee).AddPhase(durations.At(ee));
          durations.At(ee) = 0.0; // reset

        } else { // swinglegleg to stance
          endeffectors_.At(ee).AddPhase(durations.At(ee));
          durations.At(ee) = 0.0; // reset
        }
      }
    }
  }

  EndeffectorsBool swinglegs = phases.back().first;
  double T                   = phases.back().second;

  for (auto ee : swinglegs.GetEEsOrdered()) {
    if (!swinglegs.At(ee)) // last phase is stance
      endeffectors_.At(ee).AddPhase(durations.At(ee)+T);
    else
      endeffectors_.At(ee).AddPhase(durations.At(ee) + T);
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

} /* namespace opt */
} /* namespace xpp */

