/**
 @file    contact_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/contact_schedule.h>

#include <string>

#include <xpp/opt/variables/spline.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

ContactSchedule::ContactSchedule (EndeffectorID ee, const FullPhaseVec& phases)
   :Component(0, id::GetEEContactId(ee))
{
  SetPhaseSequence(phases, ee);

  SetRows(durations_.size());
}

ContactSchedule::~ContactSchedule ()
{
}

void
ContactSchedule::SetPhaseSequence (const FullPhaseVec& phases, EndeffectorID ee)
{
  double durations = 0.0;

  first_phase_in_contact_ = phases.front().first.At(ee);

  for (int i=0; i<phases.size()-1; ++i) {

    bool is_swingleg = phases.at(i).first.At(ee);
    bool is_swingleg_next = phases.at(i+1).first.At(ee);;

    durations += phases.at(i).second;

    // check if next phase is different phase
    bool next_different = is_swingleg != is_swingleg_next;

    if (next_different) {
      durations_.push_back(durations);
      durations = 0.0; // reset
    }
  }

  durations_.push_back(durations+phases.back().second); // always add last phase
}

bool
ContactSchedule::IsInContact (double t_global) const
{
  int id = Spline::GetSegmentID(t_global, durations_);
  return GetContact(id);
}

VectorXd
ContactSchedule::GetValues () const
{
  return Eigen::Map<VectorXd>(durations_.data(), durations_.size());
}

void
ContactSchedule::SetValues (const VectorXd& x)
{
  VectorXd::Map(&durations_[0], x.rows()) = x;
}

VecBound
ContactSchedule::GetBounds () const
{
  VecBound bounds;

  for (auto T : durations_)
    bounds.push_back(Bound(0.1, 1.0));

  return bounds;
}

bool
ContactSchedule::GetContact (int phase) const
{
   // always alternating
  if (phase%2==0)
    return first_phase_in_contact_;
  else
    return !first_phase_in_contact_;
}

std::vector<double>
ContactSchedule::GetTimePerPhase () const
{
  return durations_;
}




DurationConstraint::DurationConstraint (const OptVarsPtr& opt_vars,
                                        double T_total, int ee)
{
  SetName("DurationConstraint-" + std::to_string(ee));
  schedule_ = std::dynamic_pointer_cast<ContactSchedule>(opt_vars->GetComponent(id::GetEEContactId(ee)));
  T_total_ = T_total;

  AddOptimizationVariables(opt_vars);
  SetRows(1);
}

DurationConstraint::~DurationConstraint ()
{
}

VectorXd
DurationConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());
  for (double t_phase : schedule_->GetTimePerPhase())
    g(0) += t_phase;

  return g;
}

VecBound
DurationConstraint::GetBounds () const
{
  return VecBound(GetRows(), Bound(T_total_, T_total_));
}

void
DurationConstraint::FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const
{
  if (var_set == schedule_->GetName())
    for (int col=0; col<schedule_->GetPhaseCount(); ++col)
      jac.coeffRef(0, col) = 1.0;
}



} /* namespace opt */
} /* namespace xpp */
