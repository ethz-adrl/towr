/**
 @file    contact_timings.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 5, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/contact_timings.h>

#include <string>

#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

ContactTimings::ContactTimings (int ee, const TimingsVec& t)
    :Component(0, id::contact_timings + std::to_string(ee))
{
//  int n_phases = 2*max_num_steps;
//  double t_total = 3.0; // just an estimate
//  TimingsVec(n_phases, t_total/n_phases);

  t_vec_ = t;
  SetRows(t.size());
}

ContactTimings::~ContactTimings ()
{
  // TODO Auto-generated destructor stub
}

VectorXd
xpp::opt::ContactTimings::GetValues () const
{
  return VectorXd::Map(t_vec_.data(), t_vec_.size());
}

void
xpp::opt::ContactTimings::SetValues (const VectorXd& x)
{
  VectorXd::Map(t_vec_.data(), x.size()) = x;
}

VecBound
ContactTimings::GetBounds () const
{
  VecBound bounds(GetRows());
  std::fill(bounds.begin(), bounds.end(), Bound(0.0, t_max_));

  return bounds;
}

} /* namespace opt */
} /* namespace xpp */

