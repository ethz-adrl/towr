/*
 * TotalDurationConstraint.h
 *
 *  Created on: Jan 7, 2018
 *      Author: winklera
 */

#ifndef XPP_OPT_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_TOTAL_DURATION_CONSTRAINT_H_
#define XPP_OPT_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_TOTAL_DURATION_CONSTRAINT_H_

#include <ifopt/leaves.h>

#include <xpp_opt/variables/contact_schedule.h>

namespace xpp {

class TotalDurationConstraint : public opt::ConstraintSet {
public:

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

} // namespace xpp

#endif /* XPP_OPT_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_TOTAL_DURATION_CONSTRAINT_H_ */
