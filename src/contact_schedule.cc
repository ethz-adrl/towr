/**
 @file    contact_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#include <xpp/variables/contact_schedule.h>

#include <cassert>
#include <iostream>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/state.h>
#include <xpp/variables/spline.h>
#include <xpp/variables/variable_names.h>


namespace xpp {
namespace opt {


ContactSchedule::ContactSchedule (EndeffectorID ee, const VecDurations& timings,
                                  double min_duration, double max_duration)
    :Component(0, id::GetEEScheduleId(ee))
{
  durations_ = timings;
  phase_duration_bounds_ = Bound(min_duration, max_duration);
  first_phase_in_contact_ = true;
  t_total_   = std::accumulate(durations_.begin(), durations_.end(), 0.0);
  SetRows(durations_.size()-1); // since last phase-duration is not optimized over
}

ContactSchedule::~ContactSchedule ()
{
}

void
ContactSchedule::AddObserver (const PhaseNodesPtr& o)
{
  observers_.push_back(o);
  UpdateObservers();
}

void
ContactSchedule::UpdateObservers () const
{
  for (auto& o : observers_)
    o->UpdateDurations(durations_);
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
  VectorXd x(GetRows());

  for (int i=0; i<x.rows(); ++i)
    x(i) = durations_.at(i);

  return x;
}

void
ContactSchedule::SetValues (const VectorXd& x)
{
  VecDurations opt_durations;

  for (int i=0; i<x.rows(); ++i)
    opt_durations.push_back(x(i));

  durations_ = CalcAllDurations(opt_durations);
  UpdateObservers();
}



ContactSchedule::VecDurations
ContactSchedule::CalcAllDurations (const VecDurations& opt_Ts) const
{
  VecDurations all_Ts;

  for (double T : opt_Ts)
    all_Ts.push_back(T);

  // includes last phase that is not optimized
  double T_last = t_total_ - std::accumulate(opt_Ts.begin(), opt_Ts.end(), 0.0);
  assert(T_last > 0.0);
  all_Ts.push_back(T_last);

  return all_Ts;
}

VecBound
ContactSchedule::GetBounds () const
{
  VecBound bounds;

  for (int i=0; i<GetRows(); ++i)
    bounds.push_back(phase_duration_bounds_);

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

std::vector<bool>
ContactSchedule::GetContactSequence () const
{
  std::vector<bool> contact_sequence_;
  for (int phase=0; phase<durations_.size(); ++phase)
    contact_sequence_.push_back(GetContact(phase));

  return contact_sequence_;
}

std::vector<double>
ContactSchedule::GetTimePerPhase () const
{
  return durations_;
}

Jacobian
ContactSchedule::GetJacobianOfPos (double t_global, const std::string& id) const
{
  PhaseNodesPtr o = GetObserver(id);
  VectorXd dx_dT  = o->GetDerivativeOfPosWrtPhaseDuration(t_global);
  VectorXd xd     = o->GetPoint(t_global).v_;

  int n_dim = xd.rows();
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(n_dim, GetRows());

  int current_phase = Spline::GetSegmentID(t_global, durations_);
  bool in_last_phase = current_phase == durations_.size()-1;

  // duration of current phase expands and compressed spline
  if (!in_last_phase)
    jac.col(current_phase) = dx_dT;

  for (int phase=0; phase<current_phase; ++phase) {
    // each previous durations shifts spline along time axis
    jac.col(phase) = -1*xd;

    // in last phase previous duration cause expansion/compression of spline
    // as final time is fixed.
    if (in_last_phase)
      jac.col(phase) -= dx_dT;
  }

  // convert to sparse, but also regard 0.0 as non-zero element, because
  // could turn nonzero during the course of the program
  // as durations change and t_global falls into different spline
  return jac.sparseView(1.0, -1.0);
}

ContactSchedule::PhaseNodesPtr
ContactSchedule::GetObserver (const std::string& id) const
{
  for (const auto& o : observers_)
    if (o->GetName() == id)
      return o;

  std::cerr << "Observer \"" << id << "\" doesn't exist." << std::endl;
  assert(false);
}




DurationConstraint::DurationConstraint (const OptVarsPtr& opt_vars,
                                        double T_total, int ee)
{
  SetName("DurationConstraint-" + std::to_string(ee));
  schedule_ = std::dynamic_pointer_cast<ContactSchedule>(opt_vars->GetComponent(id::GetEEScheduleId(ee)));
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
//  for (double t_phase : schedule_->GetTimePerPhase())
//    g(0) += t_phase;
//
  g(0) = schedule_->GetValues().sum();
  return g;
}

VecBound
DurationConstraint::GetBounds () const
{
  return VecBound(GetRows(), Bound(0.1, T_total_-0.2));
}

void
DurationConstraint::FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const
{
  if (var_set == schedule_->GetName())
    for (int col=0; col<schedule_->GetRows(); ++col)
      jac.coeffRef(0, col) = 1.0;
}


double
ContactSchedule::GetTotalTime () const
{
  return std::accumulate(durations_.begin(), durations_.end(), 0.0);
}

} /* namespace opt */
} /* namespace xpp */

