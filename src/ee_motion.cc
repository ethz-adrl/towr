/**
 @file    ee_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 13, 2017
 @brief   Defines the EEMotion class.
 */

#include <xpp/opt/variables/ee_motion.h>

#include <cassert>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/opt/polynomial.h>

namespace xpp {
namespace opt {

EEMotion::EEMotion () : OptimizationVariables("ee_motion_single")
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
EEMotion::AddPhase (double t, double lift_height, bool is_contact)
{
  Contact c_prev = GetLastContact();
  Contact c_goal = c_prev;

  // endeffector is only allowed to move during swing phase, so new ID.
  if (!is_contact)
    c_goal.id++;

  EEPhaseMotion motion;
  motion.Init(t, lift_height, c_prev.p, c_goal.p);
  phase_motion_.push_back(motion);

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

VectorXd
EEMotion::GetVariables () const
{
  // Attention: remember to adapt GetJacobianPos() contact-phase part and Index()
  // when changing this...sorry.
  int id_prev = -1;
  int n_contact_ids = 1 + GetLastContact().id;

  VectorXd x(n_contact_ids*kDim2d); // initial position plus goal steps-xy

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
EEMotion::SetVariables (const VectorXd& x)
{
  for (PhaseContacts& phase: phase_contacts_)
    for (Contact& c : phase)
      for (auto dim : d2::AllDimensions)
        c.p(dim) = x(Index(c.id,dim));

  UpdateSwingMotions();
}

JacobianRow
EEMotion::GetJacobianPos (double t_global, d2::Coords dim) const
{
  JacobianRow jac(GetOptVarCount());

  // figure out which contacts affect the motion
  int phase      = GetPhase(t_global);
  double t_local = GetLocalTime(t_global, phase);

  // same id for stance phase
  int idx_start = Index(phase_contacts_.at(phase).front().id, dim);
  int idx_goal  = Index(phase_contacts_.at(phase).back().id, dim);

  jac.insert(idx_start) = phase_motion_.at(phase).GetDerivativeOfPosWrtContactsXY(dim, t_local, Polynomial::Start);
  jac.insert(idx_goal)  = phase_motion_.at(phase).GetDerivativeOfPosWrtContactsXY(dim, t_local, Polynomial::Goal);

  if (idx_start == idx_goal)// in contact phase // zmp_ ugly
    jac.insert(idx_start) = 1.0;

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
