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

#include <Eigen/Sparse>
#include <deque>
#include "ee_phase_motion.h"

namespace xpp {
namespace opt {

/** Parametrizes the motion of one(!) endeffector swinging multiple times.
  */
class EEMotion : public Parametrization {
public:
  using ContactPositions = std::deque<Contact>;
  using PhaseContacts    = std::array<Contact, 2>;

  EEMotion ();
  virtual ~EEMotion ();

  void SetInitialPos(const Vector3d& pos, EndeffectorID);
  void AddStancePhase(double t);
  void AddSwingPhase(double t, const Vector3d& goal);


  StateLin3d GetState(double t_global) const;
  double GetTotalTime() const;


  // zmp_ remove both these function
  bool IsInContact(double t_global) const;
  /** Empty vector is no contact at that time */
  ContactPositions GetContact(double t_global) const;


  VectorXd GetOptimizationParameters() const override;
  void SetOptimizationParameters(const VectorXd&) override;
  // haven't yet implemented the derivative during swing phase
  JacobianRow GetJacobianPos(double t, d2::Coords dimension) const;
  int Index(int id, d2::Coords dimension) const;


private:
  int GetPhase(double t_global) const;
  void AddPhase(double t, const Vector3d& goal, double lift_height, int id_goal);
  void UpdateSwingMotions();
  double GetLocalTime(double t_global, int phase) const;

  Contact GetLastContact() const;

//  ContactPositions contacts_;
  Contact first_contact_;
  int n_steps = 0;

  std::vector<PhaseContacts> phase_contacts_;

  std::deque<bool> is_contact_phase_; // zmp_ this deserves a separate class
  std::vector<EEPhaseMotion> phase_motion_;

};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_ */
