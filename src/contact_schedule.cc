/**
 @file    contact_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/contact_schedule.h>

#include <string>

#include <xpp/opt/variables/spline.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

ContactSchedule::ContactSchedule (EndeffectorID ee, const FullPhaseVec& phases)
   :Component(0, id::GetEEContactId(ee))
{
  SetPhaseSequence(phases, ee);

  SetRows(durations_.size());
}

ContactSchedule::~ContactSchedule ()
{
}

void
ContactSchedule::SetPhaseSequence (const FullPhaseVec& phases, EndeffectorID ee)
{
  double durations = 0.0;

  first_phase_in_contact_ = phases.front().first.At(ee);

  for (int i=0; i<phases.size()-1; ++i) {

    bool is_swingleg = phases.at(i).first.At(ee);
    bool is_swingleg_next = phases.at(i+1).first.At(ee);;

    durations += phases.at(i).second;

    // check if next phase is different phase
    bool next_different = is_swingleg != is_swingleg_next;

    if (next_different) {
      durations_.push_back(durations);
      durations = 0.0; // reset
    }
  }

  durations_.push_back(durations+phases.back().second); // always add last phase
}

bool
ContactSchedule::IsInContact (double t_global) const
{
  int id = Spline::GetSegmentID(t_global, durations_);
  return GetContact(id);
}

VectorXd
ContactSchedule::GetValues () const
{
  return Eigen::Map<VectorXd>(durations_.data(), durations_.size());
}

void
ContactSchedule::SetValues (const VectorXd& x)
{
  VectorXd::Map(&durations_[0], x.rows()) = x;
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
  return durations_;
}

} /* namespace opt */
} /* namespace xpp */
