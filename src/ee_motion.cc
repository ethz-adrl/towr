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
  // TODO Auto-generated constructor stub
}

EEMotion::~EEMotion ()
{
  // TODO Auto-generated destructor stub
}

void
EEMotion::SetInitialPos (const Vector3d& pos)
{
  contacts_.push_front(pos);
}

void
EEMotion::AddStancePhase (double t)
{
  AddPhase(t, contacts_.back(), 0.0); // stay at same position and don't lift leg
  is_contact_phase_.push_back(true);
}

void
EEMotion::AddSwingPhase (double t, const Vector3d& goal)
{
  AddPhase(t, goal);
  contacts_.push_back(goal);
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
  return is_contact_phase_.at(GetPhase(t_global));
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

void
EEMotion::AddPhase (double t, const Vector3d& goal, double lift_height)
{
  EESwingMotion swing_motion;
  swing_motion.SetDuration(t);
  swing_motion.lift_height_ = lift_height;
  swing_motion.SetContacts(contacts_.back(), goal);
  phase_motion_.push_back(swing_motion);
}

} /* namespace opt */
} /* namespace xpp */
