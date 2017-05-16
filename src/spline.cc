/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#include <cassert>

#include <xpp/opt/spline.h>
#include <xpp/state.h>

namespace xpp {
namespace opt {

double
Spline::GetTotalTime() const
{
  double T = 0.0;
  for (const auto& s: segments_) {
    T += s->GetDuration();
  }
  return T;
}

double
Spline::GetLocalTime(double t_global) const
{
  int id_spline = GetSegmentID(t_global);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= segments_.at(id)->GetDuration();
  }

  return t_local;//-eps_; // just to never get value greater than true duration due to rounding errors
}

const StateLinXd
Spline::GetPoint(double t_global) const
{
  int idx        = GetSegmentID(t_global);
  double t_local = GetLocalTime(t_global);

  return segments_.at(idx)->GetPoint(t_local);
}

int
Spline::GetSegmentID(double t_global) const
{
  double eps = 1e-10; // double imprecision
  assert(t_global<=GetTotalTime()+eps); // machine precision

   double t = 0;
   int i=0;
   for (const auto& s: segments_) {
     t += s->GetDuration();

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }
   assert(false); // this should never be reached
}

} // namespace opt
} // namespace xpp
