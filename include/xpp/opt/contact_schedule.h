/**
 @file    contact_schedule.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_

#include <xpp/endeffectors.h>
#include "../optimization_variables.h"

namespace xpp {
namespace opt {

class SingleContactMotion {
public:
  using Phase = std::pair<bool, double>; // contact state and duration
  using PhaseVec = std::vector<Phase>;

  SingleContactMotion ();
  virtual ~SingleContactMotion ();

  void SetFirstContactState(bool);
  void AddPhase(double t_duration);
  bool IsInContact(double t_global) const;

  PhaseVec GetPhases() const;

private:
  bool GetContact(int phase) const;

  bool first_phase_in_contact_ = true;
  std::vector<double> t_phase_end_; ///< global time when the contact changes.
};


/** @brief Knows which endeffectors are in contact at time t during trajectory.
 */
class ContactSchedule : public OptimizationVariables {
public:
  using EEContacts = Endeffectors<SingleContactMotion>;
  using Phase      = std::pair<EndeffectorsBool, double>; // swinglegs and time
  using PhaseVec   = std::vector<Phase>;

  ContactSchedule (const PhaseVec& phases);
  virtual ~ContactSchedule ();

  EndeffectorsBool IsInContact(double t_global) const;
  int GetContactCount(double t_global) const;


  // so far not optimizing over these
  VectorXd GetVariables() const override { return VectorXd(); };
  void SetVariables(const VectorXd&) override {};


  SingleContactMotion::PhaseVec GetPhases(EndeffectorID) const;

private:
  void SetPhaseSequence (const PhaseVec& phases);
  void SetInitialSwinglegs(const EndeffectorsBool&);

  EEContacts endeffectors_;
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_ */
