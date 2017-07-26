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

ContactSchedule::ContactSchedule (EndeffectorID ee,
                                  double t_total,
                                  const FullPhaseVec& phases)
   :Component(0, id::GetEEContactId(ee))
{
  t_total_ = t_total;
  SetPhaseSequence(phases, ee);
  SetRows(durations_.size()-1); // since last duration results from total time and previous durations
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
  VectorXd x = Eigen::Map<VectorXd>(durations_.data(), GetRows());

//  std::cout << "x: " << x.transpose() << std::endl;
  return x;
}

void
ContactSchedule::SetValues (const VectorXd& x)
{
  VectorXd::Map(&durations_[0], x.rows()) = x;
  double t_last = t_total_ - x.sum();
  durations_.back() = t_last;

//  std::cout << "x.rows(): " << x.rows() << std::endl;
//  std::cout << "x.sum(): " << x.sum() << std::endl;
//  std::cout << "t_last: " << t_last << std::endl;
//
//  std::cout << "durations: " << std::endl;
//  for (double d : durations_)
//    std::cout << d << std::endl;
}

VecBound
ContactSchedule::GetBounds () const
{
  double t_min = 0.1; // [s]
  double t_max = t_total_/durations_.size(); // [s] // zmp_ this is a huge constraint and should be fixed
  return VecBound(GetRows(), Bound(t_min, t_max));
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




//DurationConstraint::DurationConstraint (const OptVarsPtr& opt_vars,
//                                        double T_total, int ee)
//{
//  SetName("DurationConstraint-" + std::to_string(ee));
//  schedule_ = std::dynamic_pointer_cast<ContactSchedule>(opt_vars->GetComponent(id::GetEEContactId(ee)));
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
//  for (double t_phase : schedule_->GetTimePerPhase())
//    g(0) += t_phase;
//
//  return g;
//}
//
//VecBound
//DurationConstraint::GetBounds () const
//{
//  return VecBound(GetRows(), Bound(T_total_, T_total_));
//}
//
//void
//DurationConstraint::FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const
//{
//  if (var_set == schedule_->GetName())
//    for (int col=0; col<schedule_->GetPhaseCount(); ++col)
//      jac.coeffRef(0, col) = 1.0;
//}



} /* namespace opt */
} /* namespace xpp */
