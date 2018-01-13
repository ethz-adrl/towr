/*
 * TotalDurationConstraint.h
 *
 *  Created on: Jan 7, 2018
 *      Author: winklera
 */

#ifndef TOWR_CONSTRAINTS_TOTAL_DURATION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_TOTAL_DURATION_CONSTRAINT_H_

#include <ifopt/constraint_set.h>

#include <towr/variables/contact_schedule.h>

namespace towr {

class TotalDurationConstraint : public ifopt::ConstraintSet {
public:
  using EndeffectorID = uint;

  TotalDurationConstraint(double T_total, int ee);
  ~TotalDurationConstraint() = default;

  virtual void InitVariableDependedQuantities(const VariablesPtr& x) override;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:
  ContactSchedule::Ptr schedule_;
  double T_total_;
  EndeffectorID ee_;
};

} // namespace towr

#endif /* TOWR_CONSTRAINTS_TOTAL_DURATION_CONSTRAINT_H_ */
