/**
 @file    contact_schedule.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#ifndef TOWR_VARIABLES_CONTACT_SCHEDULE_H_
#define TOWR_VARIABLES_CONTACT_SCHEDULE_H_

#include <memory>
#include <string>
#include <vector>

#include <xpp_states/endeffectors.h>

#include <ifopt/composite.h>

#include "spline.h"

namespace towr {

class ContactSchedule : public ifopt::VariableSet {
public:
  using Ptr           = std::shared_ptr<ContactSchedule>;
  using VecDurations  = std::vector<double>;
  using EndeffectorID = xpp::EndeffectorID;

  ContactSchedule (EndeffectorID ee,
                   const VecDurations& timings,
                   double min_phase_duration,
                   double max_phase_duration);
  virtual ~ContactSchedule () = default;

  VecDurations GetDurations() const { return durations_; };


  void AddObserver(const Spline::Ptr& o);
  void UpdateObservers() const;

  virtual VectorXd GetValues() const override;
  virtual void SetVariables(const VectorXd&) override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianOfPos(double t_global, const std::string& observer_name) const;


private:
  double t_total_;
  ifopt::Bounds phase_duration_bounds_;

  std::vector<Spline::Ptr> observers_;
  Spline::Ptr GetObserver(const std::string& id) const;

  VecDurations durations_;
};



} /* namespace towr */

#endif /* TOWR_VARIABLES_CONTACT_SCHEDULE_H_ */
