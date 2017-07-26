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


class ContactSchedule : public Component {
public:
  using Phase = std::pair<bool, double>; // contact state and duration
  using PhaseVec = std::vector<Phase>;
  using FullPhase    = std::pair<EndeffectorsBool, double>; // swinglegs and time
  using FullPhaseVec = std::vector<FullPhase>;

  ContactSchedule (EndeffectorID ee, const FullPhaseVec& phases);
  virtual ~ContactSchedule ();

  bool IsInContact(double t_global) const;

  std::vector<double> GetTimePerPhase() const;

  virtual VectorXd GetValues() const override { return VectorXd(); };
  virtual void SetValues(const VectorXd&) override {};

private:
  void SetPhaseSequence (const FullPhaseVec& phases, EndeffectorID ee);
  void AddPhase(double t_duration);
  bool GetContact(int phase) const;
  PhaseVec GetPhases() const;

  bool first_phase_in_contact_ = true;
  std::vector<double> t_phase_end_; ///< global time when the contact changes.
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_ */
