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

ContactTimings::ContactTimings (int ee, int max_num_steps)
    :Component(0, id::contact_timings + std::to_string(ee))
{
  int n_phases = 2*max_num_steps;
  t_ = VectorXd(n_phases); //as a step is always followed by a stance
  double t_total = 3.0; // just an estimate
  t_.fill(t_total/n_phases);

  SetRows(n_phases);
}

ContactTimings::~ContactTimings ()
{
  // TODO Auto-generated destructor stub
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
