/**
 @file    contact_schedule.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_

#include <memory>
#include <string>
#include <vector>

#include <xpp_states/endeffectors.h>

#include <xpp/composite.h>
#include <xpp/nlp_bound.h>

#include "phase_nodes.h"


namespace xpp {
namespace opt {


class ContactSchedule : public Component {
public:
  using Ptr           = std::shared_ptr<ContactSchedule>;
  using VecDurations  = std::vector<double>;
  using PhaseNodesPtr = std::shared_ptr<PhaseNodes>;

  ContactSchedule (EndeffectorID ee,
                   double t_total,
                   const VecDurations& timings,
                   bool is_in_contact_at_start,
                   double min_phase_duration,
                   double max_phase_duration);
  virtual ~ContactSchedule ();

  bool IsInContact(double t_global) const;

  std::vector<double> GetTimePerPhase() const;
  std::vector<bool> GetContactSequence() const;

  void AddObserver(const PhaseNodesPtr& o);
  void UpdateObservers() const;

  virtual VectorXd GetValues() const override;
  virtual void SetValues(const VectorXd&) override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianOfPos(double t_global, const std::string& observer_name) const;

  int GetPhaseCount() const { return GetTimePerPhase().size(); };
  bool GetContact(int phase) const;

  double GetTotalTime() const;

private:
  bool first_phase_in_contact_;
  double t_total_;
  NLPBound phase_duration_bounds_;

  std::vector<PhaseNodesPtr> observers_;
  PhaseNodesPtr GetObserver(const std::string& id) const;

  VecDurations durations_;




};



/** Makes sure all phase durations sum up to final specified motion duration.
 */
class DurationConstraint : public Primitive {
public:
  using SchedulePtr = std::shared_ptr<ContactSchedule>;

  DurationConstraint(const OptVarsPtr& opt_vars, double T_total, int ee);
  ~DurationConstraint();

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;

private:
  SchedulePtr schedule_;
  double T_total_;
};




} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_ */
