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

StateLin3d
EEMotion::GetState (double t_global) const
{
  // zmp_ continue here, DRY with function below, find smart way to do this
  double t = 0;
  for (int i=0; i<contacts_.size(); ++i) {

    t += timings_.at(i).stance;
    if (t >= t_global)
      return StateLin3d(contacts_.at(i));

    double t_local = t_global - t;
    t += timings_.at(i).swing;
    if (t >= t_global) {
      return swing_motions_.at(i).GetState(t_local);
    }
  }
}

bool
EEMotion::IsInContact (double t_global) const
{
  double t = 0;
  for (Tlocal t_local : timings_) {

    t += t_local.stance;
    if (t >= t_global)
      return true;

    t += t_local.swing;
    if (t >= t_global)
      return false;
  }
}

void
EEMotion::SetParameters (const Timings& timings, const Contacts& contacts)
{
  contacts_ = contacts;
  timings_ = timings;

  for (int i=0; i<contacts.size()-1; ++i) {
    Vector3d contact = contacts.at(i);
    Tlocal   t_local = timings.at(i);

    EESwingMotion swing_motion;
    swing_motion.SetDuration(timings.at(i).swing);
    swing_motion.SetContacts(contacts.at(i), contacts.at(i+1));

    swing_motions_.push_back(swing_motion);
  }
}

} /* namespace opt */
} /* namespace xpp */
