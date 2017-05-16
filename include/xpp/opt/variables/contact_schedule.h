/**
 @file    contact_schedule.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_

#include <Eigen/Dense>
#include <utility>
#include <vector>

#include <xpp/endeffectors.h>

#include <xpp/opt/constraints/composite.h>

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
  double GetTotalTime() const { return t_phase_end_.back(); };

private:
  bool GetContact(int phase) const;

  bool first_phase_in_contact_ = true;
  std::vector<double> t_phase_end_; ///< global time when the contact changes.
};


/** @brief Knows which endeffectors are in contact at time t during trajectory.
 */
class ContactSchedule : public Component {
public:
  using EEContacts = Endeffectors<SingleContactMotion>;
  using Phase      = std::pair<EndeffectorsBool, double>; // swinglegs and time
  using PhaseVec   = std::vector<Phase>;

  ContactSchedule (const PhaseVec& phases);
  virtual ~ContactSchedule ();

  EndeffectorsBool IsInContact(double t_global) const;
  int GetContactCount(double t_global) const;

  double GetTotalTime() const;


  // so far not optimizing over these
  virtual VectorXd GetValues() const override { return VectorXd(); };
  virtual void SetValues(const VectorXd&) override {};


  SingleContactMotion::PhaseVec GetPhases(EndeffectorID) const;

private:
  EEContacts endeffectors_;
  void SetPhaseSequence (const PhaseVec& phases);
  void SetInitialSwinglegs(const EndeffectorsBool&);

};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_ */
