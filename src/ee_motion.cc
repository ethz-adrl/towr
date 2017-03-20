/**
 @file    ee_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 13, 2017
 @brief   Defines the EEMotion class.
 */

#include <xpp/opt/ee_motion.h>

namespace xpp {
namespace opt {

EEMotion::EEMotion ()
{
}

EEMotion::~EEMotion ()
{
  // TODO Auto-generated destructor stub
}

// zmp_ have a temporal dependency here on AddStace/Swing phase, maybe save all
// in vector first and then generate splines?

void
EEMotion::SetInitialPos (const Vector3d& pos, EndeffectorID ee)
{
  // zmp_ then also don't clear this
  is_contact_phase_.clear();
  contacts_.clear();
  phase_motion_.clear();

  ee_ = ee;
  contacts_.push_front(Contact(ContactBase::kFixedByStartStance, ee_, pos));
  UpdateSwingMotions();
}

void
EEMotion::AddStancePhase (double t)
{
  AddPhase(t, contacts_.back().p, 0.0); // stay at same position and don't lift leg
  is_contact_phase_.push_back(true);
}

void
EEMotion::AddSwingPhase (double t, const Vector3d& goal)
{
  AddPhase(t, goal);
//  int contact_nr = contacts_.size() + ContactBase::kFixedByStartStance;
  Contact c(contacts_.size(), ee_, goal);
  contacts_.push_back(c);
  is_contact_phase_.push_back(false);
}

StateLin3d
EEMotion::GetState (double t_global) const
{
  int phase = GetPhase(t_global);
  double t_local = t_global;
  for (int i=0; i<phase; ++i)
    t_local -= phase_motion_.at(i).GetDuration();

  return phase_motion_.at(phase).GetState(t_local);
}

bool
EEMotion::IsInContact (double t_global) const
{
  int phase = GetPhase(t_global);
  return is_contact_phase_.at(phase);
}

int
EEMotion::GetPhase (double t_global) const
{
  double t = 0.0;
  for (int i=0; i<phase_motion_.size(); ++i) {
    t += phase_motion_.at(i).GetDuration();
    if (t >= t_global)
      return i;
  }
}

bool
EEMotion::IsInContact (double t_global) const
{
  int phase = GetPhase(t_global);
  return is_contact_phase_.at(phase);
}

EEMotion::ContactPositions
EEMotion::GetContact (double t) const
{
  ContactPositions contact;
  if (IsInContact(t)) {

    // add up all swing phases until then
    int previous_swing_phases = 0;
    for (int p=0; p<GetPhase(t); ++p)
      previous_swing_phases += !is_contact_phase_.at(p);

    contact.push_back(contacts_.at(previous_swing_phases));
  }

  return contact;
}

void
EEMotion::UpdateSwingMotions ()
{
  int k=0; // contact
  int i=0; // phase

  for (auto& p : phase_motion_) {
    if (is_contact_phase_.at(i++))
      p.SetContacts(contacts_.at(k).p, contacts_.at(k).p);
    else {
      p.SetContacts(contacts_.at(k).p, contacts_.at(k+1).p);
      k++;
    }
  }
}

void
EEMotion::AddPhase (double t, const Vector3d& goal, double lift_height)
{
  EESwingMotion swing_motion;
  swing_motion.SetDuration(t);
  swing_motion.lift_height_ = lift_height;
  swing_motion.SetContacts(contacts_.back().p, goal);
  phase_motion_.push_back(swing_motion);
}

EEMotion::ContactPositions
EEMotion::GetFreeContacts () const
{
  return ContactPositions(contacts_.begin()+1, contacts_.end());
}

void
EEMotion::SetContactPosition (int foothold_of_leg, const Vector3d& pos)
{
  contacts_.at(foothold_of_leg).p = pos;
  UpdateSwingMotions();
}

double
EEMotion::GetTotalTime () const
{
  double T = 0.0;
  for (auto p : phase_motion_) {
    T += p.GetDuration();
  }

  return T;
}

} /* namespace opt */
} /* namespace xpp */

