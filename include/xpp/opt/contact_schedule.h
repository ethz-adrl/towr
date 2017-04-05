/**
 @file    contact_schedule.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_

#include <xpp/endeffectors.h>
#include <xpp/parametrization.h>

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
class ContactSchedule : public Parametrization {
public:
  using EEContacts = Endeffectors<SingleContactMotion>;
  using Phase      = std::pair<EndeffectorsBool, double>; // swinglegs and time
  using PhaseVec   = std::vector<Phase>;

  ContactSchedule ();
  virtual ~ContactSchedule ();

  void SetPhaseSequence (const PhaseVec& phases);
  EndeffectorsBool IsInContact(double t_global) const;
  int GetContactCount(double t_global) const;


  // so far not optimizing over these
  VectorXd GetOptimizationParameters() const override { return VectorXd(); };
  void SetOptimizationParameters(const VectorXd&) override {};


private:
  void SetInitialSwinglegs(const EndeffectorsBool&);

  EEContacts endeffectors_;
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_ */
