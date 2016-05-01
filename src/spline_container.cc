/**
@file    spline_container.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Holds coefficients of multiple fifth-order splines and returns state
         (pos,vel,acc) of appropriate spline for a specific time
 */

#include <xpp/zmp/spline_container.h>
#include <xpp/hyq/support_polygon_container.h>

#include <boost/range/adaptor/reversed.hpp> // boost::adaptors::reverse
#include <cmath>
#include <iostream>
#include <iomanip>      // std::setprecision
#include <cassert>

namespace xpp {
namespace zmp {

SplineContainer::SplineContainer(const VecSpline& splines)
    :splines_(splines),
     splines_initialized_(true)
{}


SplineContainer::SplineContainer(const VecLegID& step_sequence,
                                 double t_stance,
                                 double t_swing,
                                 double t_stance_initial,
                                 double t_stance_final)
{
  Init(step_sequence,t_stance,t_swing,t_stance_initial,t_stance_final);
}


void SplineContainer::Init(const std::vector<xpp::hyq::LegID>& step_sequence,
                           double t_stance,
                           double t_swing,
                           double t_stance_initial,
                           double t_stance_final)
{
  splines_ = ConstructSplineSequence(step_sequence,t_stance,t_swing,t_stance_initial,t_stance_final);
  splines_initialized_ = true;
}


// Creates a sequence of Splines without the optimized coefficients
SplineContainer::VecSpline
SplineContainer::ConstructSplineSequence(
    const std::vector<LegID>& step_sequence,
    double t_stance,
    double t_swing,
    double t_stance_initial,
    double t_stance_final)
{
  VecSpline splines;
  int step = 0;
  unsigned int id = 0; // unique identifiers for each spline

  // first 4ls-phase and step
  splines.push_back(ZmpSpline(id++, t_stance_initial, Initial4lsSpline, step));

  int n_steps = step_sequence.size();
  if (n_steps > 0)
    splines.push_back(ZmpSpline(id++, t_swing, StepSpline, step));

  for (int i=1; i<n_steps; ++i)
  {
    step++;

    // 1. insert 4ls-phase when switching between disjoint support triangles
    xpp::hyq::LegID swing_leg = step_sequence[i];
    xpp::hyq::LegID swing_leg_prev = step_sequence[i-1];
    if (xpp::hyq::SupportPolygonContainer::Insert4LSPhase(swing_leg_prev, swing_leg))
      splines.push_back(ZmpSpline(id++, t_stance, Intermediate4lsSpline, step));

    // insert swing phase splines
    splines.push_back(ZmpSpline(id++, t_swing, StepSpline, step));
  }

  // always have last 4ls spline for robot to move into center of feet
  if (n_steps > 0)
    splines.push_back(ZmpSpline(id++, t_stance_final, Final4lsSpline, step+1));

  return splines;
}


double SplineContainer::GetTotalTime(const VecSpline& splines, bool exclude_4ls_splines)
{
  double T = 0.0;
  for (ZmpSpline s: splines) {

    if (s.IsFourLegSupport() && exclude_4ls_splines)
      continue;

    T += s.GetDuration();
  };
  return T;
}


int SplineContainer::GetSplineID(double t_global, const VecSpline& splines)
{
   assert(t_global<=GetTotalTime(splines));

   double t = 0;
   for (ZmpSpline s: splines) {
     t += s.GetDuration();

     if (t >= t_global)
       return s.GetId();
   }
   return splines.back().GetId();
}


int SplineContainer::GetCurrOrNextStep(double t_global) const
{
  assert(t_global<=GetTotalTime());
  ZmpSpline s = splines_.at(GetSplineID(t_global));

  if (s.IsFourLegSupport())
    return s.GetNextStep();
  else
    return s.GetCurrStep();
}


int SplineContainer::GetFourLegSupport(double t_global) const
{
  assert(t_global<=GetTotalTime());
  return splines_.at(GetSplineID(t_global)).IsFourLegSupport();
}


void SplineContainer::GetCOGxy(double t_global, Point2d& cog_xy,
                               const VecSpline& splines)
{
  assert(t_global<=GetTotalTime(splines));

  uint curr_spline = 0;

  /** Transform global time to local spline time dt */
  double t_local = t_global;
  while (t_local > splines[curr_spline].GetDuration()) {
    t_local -= splines[curr_spline++].GetDuration();
  }

  cog_xy.p = splines[curr_spline].GetState(kPos, t_local);
  cog_xy.v = splines[curr_spline].GetState(kVel, t_local);
  cog_xy.a = splines[curr_spline].GetState(kAcc, t_local);

  assert(curr_spline < splines.size()); // make sure the current spline is in the buffer
}


void
SplineContainer::CheckIfSplinesInitialized() const
{
  if (!splines_initialized_) {
    throw std::runtime_error("ContinousSplineContainer.splines_ not initialized. Call Init() first");
  }
}

} // namespace zmp
} // namespace xpp
