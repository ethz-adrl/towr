/*
 * TotalDurationConstraint.cc
 *
 *  Created on: Jan 7, 2018
 *      Author: winklera
 */

#include <towr/constraints/total_duration_constraint.h>

#include <towr/variables/variable_names.h>

namespace xpp {

TotalDurationConstraint::TotalDurationConstraint (double T_total, int ee)
    :ConstraintSet(1, "DurationConstraint_ee_-" + std::to_string(ee))
{
  T_total_ = T_total;
  ee_ = ee;
}

void
TotalDurationConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  schedule_ = std::dynamic_pointer_cast<ContactSchedule>(x->GetComponent(id::GetEEScheduleId(ee_)));
}

VectorXd
TotalDurationConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());
//  for (double t_phase : schedule_->GetTimePerPhase())
//    g(0) += t_phase;
//
  g(0) = schedule_->GetValues().sum();
  return g;
}

TotalDurationConstraint::VecBound
TotalDurationConstraint::GetBounds () const
{
  return VecBound(GetRows(), opt::Bounds(0.1, T_total_-0.2));
}

void
TotalDurationConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == schedule_->GetName())
    for (int col=0; col<schedule_->GetRows(); ++col)
      jac.coeffRef(0, col) = 1.0;
}


} // namespace xpp
