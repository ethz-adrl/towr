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
  first_contact_ = Contact(0, ee, pos);
}

void
EEMotion::AddStancePhase (double t)
{
  Contact c_back = GetLastContact();
  AddPhase(t, c_back.p, 0.0, c_back.id); // stay at same position and don't lift leg
  is_contact_phase_.push_back(true);
}

void
EEMotion::AddSwingPhase (double t, const Vector3d& goal)
{
  double light_height = 0.03;
  AddPhase(t, goal, light_height, GetLastContact().id + 1);
  is_contact_phase_.push_back(false);
  n_steps++;
}

void
EEMotion::AddPhase (double t, const Vector3d& goal, double lift_height, int id_goal)
{
  Contact c_prev = GetLastContact();

  EEPhaseMotion motion;
  motion.Init(t, lift_height, c_prev.p, goal);
  phase_motion_.push_back(motion);


  Contact c_goal(id_goal, c_prev.ee, goal);
  PhaseContacts phase{c_prev, c_goal};
  phase_contacts_.push_back(phase);
}

StateLin3d
EEMotion::GetState (double t_global) const
{
  int phase      = GetPhase(t_global);
  double t_local = GetLocalTime(t_global, phase);

  return phase_motion_.at(phase).GetState(t_local);
}

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
  int phase = GetPhase(t);
  if (IsInContact(t))
    contact.push_back(phase_contacts_.at(phase).front()); // .back() would also work, b/c stance phase

  return contact;
}

VectorXd
EEMotion::GetOptimizationParameters () const
{
  int id_prev = -1;
  VectorXd x((1+n_steps)*kDim2d); // initial position plus goal steps-xy

  for (const PhaseContacts& contacts: phase_contacts_) {
    for (const Contact& c : contacts) { // only optimize over position at end of phase
      if (c.id != id_prev) {
        for (auto dim : d2::AllDimensions) {
          x(Index(c.id,dim)) = c.p(dim);
        }
      }
      id_prev = c.id;
    }
  }

  return x;
}

void
EEMotion::SetOptimizationParameters (const VectorXd& x)
{
  for (PhaseContacts& phase: phase_contacts_)
    for (Contact& c : phase)
      for (auto dim : d2::AllDimensions)
        c.p(dim) = x(Index(c.id,dim));

  UpdateSwingMotions();
}

JacobianRow
EEMotion::GetJacobianPos (double t_global, d2::Coords dimension) const
{
  JacobianRow jac(GetOptVarCount());

  assert(false);

//  // figure out which contacts affect the motion
//  int phase      = GetPhase(t_global);
//  double t_local = GetLocalTime(t_global, phase);
//  bool is_contact_phase = is_contact_phase_.at(phase);
//
//  ContactPositions relevant_contacts;
//  if (is_contact_phase) {
//    relevant_contacts.push_back(
//  }
//
//
//  for (auto dim : d2::AllDimensions) {
//    // for a contact phase it's this (same start and goal, only one contact affects it all)
//    double dpdc_start = phase_motion_.at(phase).GetDerivativeOfPosWrtContactsXY(dim, t_local, Polynomial::Start);
//
//    // for a swingphase the goal position is also relevant
//    double dpdc_goal  = phase_motion_.at(phase).GetDerivativeOfPosWrtContactsXY(dim, t_local, Polynomial::Goal);
//
//    jac(Index(c.id,dim)) = ...
//  }



  return jac;
}

int
EEMotion::Index (int id, d2::Coords dimension) const
{
  return id*kDim2d + dimension;
}

void
EEMotion::UpdateSwingMotions ()
{
  int phase = 0;
  for (auto& motion : phase_motion_) {
    auto contacts = phase_contacts_.at(phase);
    motion.SetContacts(contacts.front().p, contacts.back().p);
    phase++;
  }
}

double
EEMotion::GetTotalTime () const
{
  double T = 0.0;
  for (auto p : phase_motion_)
    T += p.GetDuration();

  return T;
}

double
EEMotion::GetLocalTime (double t_global, int phase) const
{
  double t_local = t_global;
  for (int i=0; i<phase; ++i)
    t_local -= phase_motion_.at(i).GetDuration();

  return t_local;
}

Contact
EEMotion::GetLastContact() const
{
  bool first_phase = phase_contacts_.empty();
  return first_phase? first_contact_ : phase_contacts_.back().back();
}

} /* namespace opt */
} /* namespace xpp */
