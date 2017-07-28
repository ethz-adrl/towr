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

ContactSchedule::ContactSchedule (EndeffectorID ee,
                                  double t_total,
                                  const SwingPhaseVec& phases)
   :Component(0, id::GetEEScheduleId(ee))
{
  t_total_ = t_total;
  SetPhaseSequence(phases, ee);
  int last_duration_fixed = true? 1 : 0; // spring_clean_ is 1 always the correct number?
  SetRows(durations_.size()-last_duration_fixed); // since last duration results from total time and previous durations

  std::cout << "GetRows(): " << GetRows() << std::endl;
}

ContactSchedule::~ContactSchedule ()
{
}

void
ContactSchedule::SetPhaseSequence (const SwingPhaseVec& phases, EndeffectorID ee)
{
  double durations = 0.0;

  durations_.clear();
  first_phase_in_contact_ = !phases.front().first.At(ee);

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

  if (ee == E0 || ee==E1)
    std::fill(durations_.begin(), durations_.end(), 0.4);
  else
    std::fill(durations_.begin(), durations_.end(), 0.1);

  double final_time = t_total_ - std::accumulate(durations_.begin(), durations_.end(), 0.0);
//  std::cout << "final time: " << final_time << std::endl;
  durations_.push_back(final_time); // always add last phase
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

  // zmp_ use shortform again
//  return Eigen::Map<VectorXd>(durations_.data(), GetRows());
}

void
ContactSchedule::SetValues (const VectorXd& x)
{
//  VectorXd::Map(&durations_[0], x.rows()) = x;
  for (int i=0; i<x.rows(); ++i) {
    durations_.at(i) = x(i);
  }


  double t_last = t_total_ - x.sum();
  durations_.back() = t_last;
  assert(t_last > 0.0);
}

VecBound
ContactSchedule::GetBounds () const
{
  VecBound bounds;
  double t_min = 0.1; // [s]
  // spring_clean_ very important! so last phase not negative
  double t_max = 0.2;//t_total_/durations_.size(); // [s] //

  for (int i=0; i<GetRows(); ++i) {
    // spring_clean_ for now exactly keep initial durations
    // spring_clean_ this really affects the jacobian structure somehow
//    Bound b(durations_.at(i), durations_.at(i)+0.1);
    Bound b(t_min, 2.0);
    bounds.push_back(b);
  }

  return bounds;

//
//  // spring_clean_ these bounds are very restrictive and should be fixed
//  double t_min = 0.1; // [s]
//  double t_max = t_total_/durations_.size(); // [s]
//  return VecBound(GetRows(), Bound(t_min, t_max));
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
  // spring_clean_ adapt for derivative of last polynomial
  // this seems way harder than it has to be...
  // which depends on all the others trough T - (t0+t1..+tn)


  int n_dim = current_vel.rows();

//  Jacobian jac(n_dim, GetRows()) = MatrixXd::Zero(;

  // convert to sparse, but also regard 0.0 as non-zero element, because
  // could turn nonzero during the course of the program
  // as durations change and t_global falls into different spline
  // zmp_ this should also work actually
//  Jacobian jac = Eigen::MatrixXd::Zero(n_dim, GetRows()).sparseView(1.0, -1.0);


  Jacobian jac(n_dim, GetRows());
  for (int dim=0; dim<n_dim; ++dim) {
    for (int col=0; col<jac.cols(); ++col) {
      jac.coeffRef(dim, col) = 0.00;
    }
  }


  int current_phase = Spline::GetSegmentID(t_global, durations_);


  for (int dim=0; dim<n_dim; ++dim) {
    if (current_phase == durations_.size()-1) {// t belongs to last phase

      // each previous durations expands and compresses the spline equally
      double inner_deriv = -1; // f(T1,T2,..) = T-T1-T2-T3...
      for (int phase=0; phase<current_phase; ++phase) {
        jac.coeffRef(dim, phase) = duration_deriv(dim)*inner_deriv;//*durations_.at(phase);
      }
    } else {

      // all previous phase durations shift complete current spline
      for (int phase=0; phase<current_phase; ++phase)
        jac.coeffRef(dim, phase) = -current_vel(dim);

      // current duration expands and compresses the spline
      jac.coeffRef(dim, current_phase) = duration_deriv(dim);
    }
  }

//  std::cout << "\n\nt: " << t_global << std::endl;
//  std::cout << "duration_deriv: " << duration_deriv.transpose() << std::endl;
//  std::cout << "current_vel: " << current_vel.transpose() << std::endl;
//  std::cout << "phase: " << current_phase << std::endl;
//  std::cout << "jac: " << jac.toDense() << std::endl;

  return jac;
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
  return VecBound(GetRows(), Bound(0.2, T_total_-0.2));
}

void
DurationConstraint::FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const
{
  if (var_set == schedule_->GetName())
    for (int col=0; col<schedule_->GetRows(); ++col)
      jac.coeffRef(0, col) = 1.0;
}



} /* namespace opt */
} /* namespace xpp */
