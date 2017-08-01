/**
 @file    contact_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/contact_schedule.h>

#include <cassert>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <numeric>

#include <xpp/opt/variables/spline.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {


ContactSchedule::ContactSchedule (EndeffectorID ee, const VecDurations& timings)
    :Component(0, id::GetEEScheduleId(ee))
{
  durations_ = timings;
  first_phase_in_contact_ = true;
  t_total_   = std::accumulate(durations_.begin(), durations_.end(), 0.0);
  SetRows(durations_.size()-1); // since last phase-duration is not optimized over
}

ContactSchedule::~ContactSchedule ()
{
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
  double t_min = 0.05; // [s]
  double t_max = t_total_/GetRows()-0.01; // [s]

  Bound b(t_min, t_max);
  for (int i=0; i<GetRows(); ++i)
    bounds.push_back(b);

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

Jacobian
ContactSchedule::GetJacobianOfPos (const VectorXd& duration_deriv,
                                   const VectorXd& current_vel,
                                   double t_global) const
{
  int n_dim = current_vel.rows();
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(n_dim, GetRows());

  int current_phase = Spline::GetSegmentID(t_global, durations_);
  bool last_phase = current_phase == durations_.size()-1;
  double inner_deriv_Tlast = -1.0; // f(T1,T2,..) = T-T1-T2-T3...


  // each previous durations expands and compresses the spline equally
  for (int phase=0; phase<current_phase; ++phase)
    if (last_phase)
      jac.col(phase) = duration_deriv*inner_deriv_Tlast;
    else
      jac.col(phase) = -1*current_vel;

  if (!last_phase)
    jac.col(current_phase) = duration_deriv; // spring_clean_ this is not the only change in this value!!!

  // convert to sparse, but also regard 0.0 as non-zero element, because
  // could turn nonzero during the course of the program
  // as durations change and t_global falls into different spline
  return jac.sparseView(1.0, -1.0);
}




//DurationConstraint::DurationConstraint (const OptVarsPtr& opt_vars,
//                                        double T_total, int ee)
//{
//  SetName("DurationConstraint-" + std::to_string(ee));
//  schedule_ = std::dynamic_pointer_cast<ContactSchedule>(opt_vars->GetComponent(id::GetEEScheduleId(ee)));
//  T_total_ = T_total;
//
//  AddOptimizationVariables(opt_vars);
//  SetRows(1);
//}
//
//DurationConstraint::~DurationConstraint ()
//{
//}
//
//VectorXd
//DurationConstraint::GetValues () const
//{
//  VectorXd g = VectorXd::Zero(GetRows());
////  for (double t_phase : schedule_->GetTimePerPhase())
////    g(0) += t_phase;
////
//  g(0) = schedule_->GetValues().sum();
//  return g;
//}
//
//VecBound
//DurationConstraint::GetBounds () const
//{
//  return VecBound(GetRows(), Bound(0.1, T_total_-0.2));
//}
//
//void
//DurationConstraint::FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const
//{
//  if (var_set == schedule_->GetName())
//    for (int col=0; col<schedule_->GetRows(); ++col)
//      jac.coeffRef(0, col) = 1.0;
//}



} /* namespace opt */
} /* namespace xpp */
