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

#include <ifopt/composite.h>
#include <xpp_opt/variables/phase_nodes.h>

namespace xpp {

class ContactSchedule : public opt::VariableSet {
public:
  using Ptr           = std::shared_ptr<ContactSchedule>;
  using VecDurations  = std::vector<double>;
  using PhaseNodesPtr = std::shared_ptr<PhaseNodes>;

  ContactSchedule (EndeffectorID ee,
                   const VecDurations& timings,
                   double min_phase_duration,
                   double max_phase_duration);
  virtual ~ContactSchedule () = default;

  void AddObserver(const PhaseNodesPtr& o);
  void UpdateObservers() const;

  virtual VectorXd GetValues() const override;
  virtual void SetVariables(const VectorXd&) override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianOfPos(double t_global, const std::string& observer_name) const;


private:
  double t_total_;
  opt::Bounds phase_duration_bounds_;

  std::vector<PhaseNodesPtr> observers_;
  PhaseNodesPtr GetObserver(const std::string& id) const;

  VecDurations durations_;
};


// spring_clean_ use own file for this
/** Makes sure all phase durations sum up to final specified motion duration.
 */
class DurationConstraint : public opt::ConstraintSet {
public:
  using SchedulePtr = std::shared_ptr<ContactSchedule>;

  DurationConstraint(double T_total, int ee);
  ~DurationConstraint() = default;

  virtual void InitVariableDependedQuantities(const VariablesPtr& x) override;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:
  SchedulePtr schedule_;
  double T_total_;
  EndeffectorID ee_;
};


} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONTACT_SCHEDULE_H_ */
