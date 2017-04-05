/**
 @file    contact_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CONTACT_MOTION_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CONTACT_MOTION_H_

#include <xpp/parametrization.h>
#include <xpp/endeffectors.h>

namespace xpp {
namespace opt {


class SingleContactMotion {
public:
  SingleContactMotion ();
  virtual ~SingleContactMotion ();

  void SetFirstContactState(bool);
  void AddPhase(double t_duration);
  bool IsInContact(double t_global) const;

private:
  bool first_phase_in_contact_ = true;
  std::vector<double> t_phase_end_; ///< global time when the contact changes.
};




/** @brief Knows which endeffectors are in contact at time t during trajectory.
 */
class ContactMotion : public Parametrization {
public:
  using EEContacts = Endeffectors<SingleContactMotion>;
  using Phase      = std::pair<EndeffectorsBool, double>; // swinglegs and time
  using PhaseVec   = std::vector<Phase>;

  ContactMotion ();
  virtual ~ContactMotion ();

  void SetPhaseSequence (const PhaseVec& phases);
  EndeffectorsBool IsInContact(double t_global) const;

private:
  void SetInitialSwinglegs(const EndeffectorsBool&);

  EEContacts endeffectors_;
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONTACT_MOTION_H_ */
