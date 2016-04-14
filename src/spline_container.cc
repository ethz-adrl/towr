/**
@file    spline_container.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Holds coefficients of multiple fifth-order splines and returns state
         (pos,vel,acc) of appropriate spline for a specific time
 */

#include <xpp/zmp/spline_container.h>
#include <xpp/hyq/support_polygon_container.h>

#include <cmath>
#include <iostream>
#include <iomanip>      // std::setprecision
#include <cassert>

namespace xpp {
namespace zmp {

SplineContainer::SplineContainer()
{
  splines_.clear();
}


double SplineContainer::GetTotalTime(const Splines& splines, bool exclude_4ls_splines) const
{
  double T = 0.0;
  for (ZmpSpline s: splines) {

    if (s.four_leg_supp_ && exclude_4ls_splines)
      continue;

    T += s.duration_;
  };
  return T;
}


int SplineContainer::GetSplineID(double t_global) const
{
   assert(t_global<=GetTotalTime());

   double t = 0;
   for (ZmpSpline s: splines_) {
     t += s.duration_;

     if (t >= t_global)
       return s.id_;
   }
}


int SplineContainer::GetStep(double t_global) const
{
  assert(t_global<=GetTotalTime());
  return splines_.at(GetSplineID(t_global)).step_;
}


int SplineContainer::GetFourLegSupport(double t_global) const
{
  assert(t_global<=GetTotalTime());
  return splines_.at(GetSplineID(t_global)).four_leg_supp_;
}


void SplineContainer::AddSpline(const ZmpSpline &spline)
{
  splines_.push_back(spline);
}


// Creates a sequence of Splines without the optimized coefficients
void SplineContainer::ConstructSplineSequence(
    const std::vector<LegID>& step_sequence,
    double t_stance,
    double t_swing,
    double t_stance_initial,
    double t_stance_final)
{
  splines_.clear();
  int step = 0;
  unsigned int id = 0; // unique identifiers for each spline

  // first 4ls-phase and step
  AddSpline(ZmpSpline(id++, t_stance_initial, true, step));
  AddSpline(ZmpSpline(id++, t_swing, false, step));

  for (size_t i = 1; i < step_sequence.size(); ++i)
  {
    step++;

    // 1. insert 4ls-phase when switching between disjoint support triangles
    xpp::hyq::LegID swing_leg = step_sequence[i];
    xpp::hyq::LegID swing_leg_prev = step_sequence[i-1];
    if (xpp::hyq::SupportPolygonContainer::Insert4LSPhase(swing_leg_prev, swing_leg))
      AddSpline(ZmpSpline(id++, t_stance, true, step));

    // insert swing phase splines
    AddSpline(ZmpSpline(id++, t_swing, false, step));
  }

  // always have last 4ls spline for robot to move into center of feet
  AddSpline(ZmpSpline(id++, t_stance_final, true, step+1));
}



void SplineContainer::GetCOGxy(double t_global, Point2d& cog_xy,
                               const Splines& splines)
{
  assert(t_global<=GetTotalTime(splines));

  uint curr_spline = 0;

  /** Transform global time to local spline time dt */
  double t_local = t_global;
  while (t_local > splines[curr_spline].duration_) {
    t_local -= splines[curr_spline++].duration_;
  }

  cog_xy.p = splines[curr_spline].GetState(kPos, t_local);
  cog_xy.v = splines[curr_spline].GetState(kVel, t_local);
  cog_xy.a = splines[curr_spline].GetState(kAcc, t_local);

  assert(curr_spline < splines.size()); // make sure the current spline is in the buffer
}

} // namespace zmp
} // namespace xpp
