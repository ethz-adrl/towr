/**
 @file    contact_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#include <../include/xpp_opt/variables/contact_schedule.h>

#include <cassert>
#include <iostream>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <../include/xpp_opt/variables/spline.h>
#include <../include/xpp_opt/variables/variable_names.h>
#include <xpp_states/state.h>



namespace xpp {


ContactSchedule::ContactSchedule (EndeffectorID ee,
                                  double t_total,
                                  const VecDurations& timings,
                                  bool is_in_contact_at_start,
                                  double min_duration,
                                  double max_duration)
    // -1 since last phase-duration is not optimized over
    :Variables(timings.size()-1, id::GetEEScheduleId(ee))
{
  t_total_   = t_total;
  for (auto d : timings)
    durations_.push_back(d*t_total);

  phase_duration_bounds_ = opt::Bounds(min_duration, max_duration);
  first_phase_in_contact_ = is_in_contact_at_start;
//  SetRows(durations_.size()-1); //
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
  for (int i=0; i<GetRows(); ++i)
    durations_.at(i) = x(i);

  durations_.back() =  t_total_ - x.sum();
  assert(durations_.back()>0);
  UpdateObservers();
}

ContactSchedule::VecBound
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

ContactSchedule::Jacobian
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




DurationConstraint::DurationConstraint (const VariablesPtr& opt_vars,
                                        double T_total, int ee)
    :Constraint(opt_vars, 1, "DurationConstraint-" + std::to_string(ee))
{
//  SetName("DurationConstraint-" + std::to_string(ee));
  schedule_ = std::dynamic_pointer_cast<ContactSchedule>(opt_vars->GetComponent(id::GetEEScheduleId(ee)));
  T_total_ = T_total;

//  AddOptimizationVariables(opt_vars);
//  SetRows(1);
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

DurationConstraint::VecBound
DurationConstraint::GetBounds () const
{
  return VecBound(GetRows(), opt::Bounds(0.1, T_total_-0.2));
}

void
DurationConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
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

} /* namespace xpp */

