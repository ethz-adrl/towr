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

EEMotion::EEMotion () : Component(-1, "ee_motion_single")
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

  auto motion = std::make_shared<EEPhaseMotion>();
  motion->Init(t, lift_height, c_prev.p, c_goal.p);
  phase_motion_.push_back(motion);
  spline_.SetSegmentsPtr(phase_motion_);

  PhaseContacts phase{c_prev, c_goal};
  phase_contacts_.push_back(phase);

  // update optimization variable count b/c new phase has been added.
  int n_contact_ids = 1 + c_goal.id;
  SetRows(n_contact_ids*kDim2d);

}

StateLin3d
EEMotion::GetState (double t_global) const
{
  return spline_.GetPoint(t_global);
}

VectorXd
EEMotion::GetValues () const
{
  // Attention: remember to adapt GetJacobianPos() contact-phase part and Index()
  // when changing this...sorry.
  int id_prev = -1;
  VectorXd x(GetRows()); // initial position plus goal steps-xy

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
EEMotion::SetValues (const VectorXd& x)
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
  JacobianRow jac(GetRows());

  // figure out which contacts affect the motion
  int phase      = spline_.GetSegmentID(t_global);
  double t_local = spline_.GetLocalTime(t_global);

  // same id for stance phase
  int idx_start = Index(phase_contacts_.at(phase).front().id, dim);
  int idx_goal  = Index(phase_contacts_.at(phase).back().id, dim);

  jac.insert(idx_start) = phase_motion_.at(phase)->GetDerivativeOfPosWrtContactsXY(dim, t_local, Polynomial::Start);
  jac.insert(idx_goal)  = phase_motion_.at(phase)->GetDerivativeOfPosWrtContactsXY(dim, t_local, Polynomial::Goal);

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
    motion->SetContacts(contacts.front().p, contacts.back().p);
    phase++;
  }
}

Contact
EEMotion::GetLastContact() const
{
  bool first_phase = phase_contacts_.empty();
  return first_phase? first_contact_ : phase_contacts_.back().back();
}

} /* namespace opt */
} /* namespace xpp */
