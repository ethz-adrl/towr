/**
 @file    contact_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/contact_motion.h>

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



ContactMotion::ContactMotion () : Parametrization("Contact Motion")
{
}

ContactMotion::~ContactMotion ()
{
}

void
ContactMotion::SetPhaseSequence (const PhaseVec& phases)
{

  for (auto ee : endeffectors_.GetEEsOrdered()) {
//    for (auto phase : phases) {
//
//      bool swing = phase.first.
//    }
  }





}

EndeffectorsBool
ContactMotion::IsInContact (double t_global) const
{

}

} /* namespace opt */
} /* namespace xpp */

