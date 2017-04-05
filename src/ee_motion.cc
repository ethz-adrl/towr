/**
 @file    ee_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 13, 2017
 @brief   Defines the EEMotion class.
 */

#include <xpp/opt/ee_motion.h>

namespace xpp {
namespace opt {

EEMotion::EEMotion () : Parametrization("ee_motion_single")
{
}

EEMotion::~EEMotion ()
{
}

void
EEMotion::SetInitialPos (const Vector3d& pos, EndeffectorID ee)
{
  ee_ = ee;
  contacts_.push_front(Contact(0, ee_, pos));
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
  Contact c(contacts_.back().id +1 , ee_, goal);
  contacts_.push_back(c);
  is_contact_phase_.push_back(false);
}

void
EEMotion::AddPhase (double t, const Vector3d& goal, double lift_height)
{
  assert(!contacts_.empty()); // SetInitialPos() must be called before

  EESwingMotion motion;
  motion.Init(t, lift_height, contacts_.back().p, goal);
  phase_motion_.push_back(motion);
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

//EEMotion::JacobianRow
//EEMotion::GetJacobianPos (double t, d2::Coords dimension) const
//{
//  assert(false); // implement
//}

int
EEMotion::GetPhase (double t_global) const
{
  double eps = 1e-10; // to ensure that last phases is returned at T
  double t = 0.0;
  for (int i=0; i<phase_motion_.size(); ++i) {
    t += phase_motion_.at(i).GetDuration();
    if (t+eps >= t_global)
      return i;
  }

  assert(false); // t_global is longer than trajectory lasts
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

EndeffectorID
EEMotion::GetEE () const
{
  return ee_;
}

VectorXd
EEMotion::GetOptimizationParameters () const
{
  // only optimize over contact xy positions for now
  VectorXd x(contacts_.size()*kDim2d);
  for (const auto& c: contacts_)
    for (auto dim : d2::AllDimensions)
      x(Index(c.id,dim)) = c.p(dim);

  return x;
}

void
EEMotion::SetOptimizationParameters (const VectorXd& x)
{
  for (auto& c: contacts_)
    for (auto dim : d2::AllDimensions)
      c.p(dim) = x(Index(c.id,dim));

  UpdateSwingMotions();
}

int
EEMotion::Index (int id, d2::Coords dimension) const
{
  return id*kDim2d + dimension;
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

// zmp_ possibly don't need this either
EEMotion::ContactPositions
EEMotion::GetContacts () const
{
  return contacts_;
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
