/**
 @file    ee_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 13, 2017
 @brief   Declares the EEMotion class.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_

#include <xpp/contact.h>
#include <xpp/parametrization.h>
#include <xpp/opt/ee_phase_motion.h>

#include <deque>

namespace xpp {
namespace opt {

/** Parameterizes the motion of one(!) endeffector over multiple phases.
  *
  * This class is purely responsible for the pos,vel,acc of the endeffectors
  * and should not know anything about the current contact state.
  */
class EEMotion : public Parametrization {
public:
  using ContactPositions = std::deque<Contact>;
  /** contact at beginning and end of phase (same for stance phase) */
  using PhaseContacts    = std::array<Contact, 2>;

  EEMotion ();
  virtual ~EEMotion ();

  void SetInitialPos(const Vector3d& pos, EndeffectorID);
  void AddPhase(double t, double lift_height, bool is_contact);

  StateLin3d GetState(double t_global) const;
  double GetTotalTime() const;

  VectorXd GetVariables() const override;
  void SetVariables(const VectorXd&) override;
  JacobianRow GetJacobianPos(double t, d2::Coords dimension) const;

private:
  int GetPhase(double t_global) const;
  void UpdateSwingMotions();
  double GetLocalTime(double t_global, int phase) const;
  int Index(int id, d2::Coords dimension) const;

  Contact GetLastContact() const;

  Contact first_contact_;

  std::vector<PhaseContacts> phase_contacts_;
  std::vector<EEPhaseMotion> phase_motion_;

};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_ */
