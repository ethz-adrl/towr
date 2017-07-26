/**
 @file    contact_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/contact_schedule.h>

#include <cassert>
#include <string>

#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

ContactSchedule::ContactSchedule (EndeffectorID ee, const FullPhaseVec& phases)
   :Component(0, id::GetEEContactId(ee))
{
  SetPhaseSequence(phases, ee);
}

ContactSchedule::~ContactSchedule ()
{
}

void
ContactSchedule::SetPhaseSequence (const FullPhaseVec& phases, EndeffectorID ee)
{
  double durations = 0.0;

  first_phase_in_contact_ = phases.front().first.At(ee);

  for (int i=0; i<phases.size(); ++i) {

    bool is_swingleg = phases.at(i).first.At(ee);
    bool is_swingleg_next;

    bool last_phase = (i==phases.size()-1);
    if (last_phase)
      is_swingleg_next = !is_swingleg; // to make sure last phase is always be added
    else
      is_swingleg_next = phases.at(i+1).first.At(ee);


    durations += phases.at(i).second;

    // check if next phase is different phase
    bool next_different = is_swingleg != is_swingleg_next;

    if (next_different) {
      AddPhase(durations);
      durations = 0.0; // reset
    }
  }
}

void
ContactSchedule::AddPhase (double t_duration)
{
  if (t_phase_end_.empty())
    t_phase_end_.push_back(t_duration);
  else // global time
    t_phase_end_.push_back(t_phase_end_.back() + t_duration);
}

bool
ContactSchedule::IsInContact (double t_global) const
{
  double eps = 1e-10;   // to ensure that last phases is returned at T

  for (int p=0; p<t_phase_end_.size(); ++p)
    if (t_phase_end_.at(p)+eps >= t_global)
      return GetContact(p);

  assert(false); // t_global longer than trajectory
}

bool
ContactSchedule::GetContact (int phase) const
{
   // always alternating
  if (phase%2==0)
    return first_phase_in_contact_;
  else
    return !first_phase_in_contact_;
}

std::vector<double>
ContactSchedule::GetTimePerPhase () const
{
  std::vector<double> T;

  for (auto p : GetPhases())
    T.push_back(p.second);

  return T;
}


ContactSchedule::PhaseVec
ContactSchedule::GetPhases () const
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

} /* namespace opt */
} /* namespace xpp */
