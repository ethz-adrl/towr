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

#include <xpp/endeffectors.h>
#include <xpp/opt/bound.h>
#include <xpp/opt/constraints/composite.h>

#include "node_values.h"


namespace xpp {
namespace opt {


class ContactSchedule : public Component {
public:
  using VecDurations  = std::vector<double>;
  using PhaseNodesPtr = std::shared_ptr<PhaseNodes>;

  ContactSchedule (EndeffectorID ee, const VecDurations& timings);
  virtual ~ContactSchedule ();

  bool IsInContact(double t_global) const;

  std::vector<double> GetTimePerPhase() const;
  std::vector<bool> GetContactSequence() const;

  void AddObserver(const PhaseNodesPtr& o);
  void UpdateObservers() const;

  // zmp_ make these std::vectors?
  virtual VectorXd GetValues() const override;
  virtual void SetValues(const VectorXd&) override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianOfPos(const VectorXd& duration_deriv,
                            const VectorXd& current_vel,
                            double t_global) const;

  int GetPhaseCount() const { return GetTimePerPhase().size(); };
  bool GetContact(int phase) const;

private:
  VecDurations CalcAllDurations(const VecDurations& opt_durations) const;

  bool first_phase_in_contact_ = true;
  double t_total_;

  std::vector<PhaseNodesPtr> observers_;

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
