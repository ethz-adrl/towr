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
  splines.push_back(ZmpSpline(id++, t_stance_initial/3., Initial4lsSpline, step));
  splines.push_back(ZmpSpline(id++, t_stance_initial/3., Initial4lsSpline, step));
  splines.push_back(ZmpSpline(id++, t_stance_initial/3., Initial4lsSpline, step));
//  splines.push_back(ZmpSpline(id++, t_stance_initial/4., Initial4lsSpline, step));
//  splines.push_back(ZmpSpline(id++, t_stance_initial/9., Initial4lsSpline, step));
//  splines.push_back(ZmpSpline(id++, t_stance_initial/9., Initial4lsSpline, step));
//  splines.push_back(ZmpSpline(id++, t_stance_initial/9., Initial4lsSpline, step));
//  splines.push_back(ZmpSpline(id++, t_stance_initial/9., Initial4lsSpline, step));
//  splines.push_back(ZmpSpline(id++, t_stance_initial/9., Initial4lsSpline, step));

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
  const uint NO_NEXT_STEP = std::numeric_limits<uint>::max();
  if (n_steps > 0)
    splines.push_back(ZmpSpline(id++, t_stance_final, Final4lsSpline, NO_NEXT_STEP));

  return splines;
}


double SplineContainer::GetTotalTime(const VecSpline& splines, bool exclude_4ls_splines)
{
  double T = 0.0;
  for (const ZmpSpline& s: splines) {

    if (s.IsFourLegSupport() && exclude_4ls_splines)
      continue;

    T += s.GetDuration();
  };
  return T-eps_; // just to never get value greater than true duration due to rounding errors
}


int SplineContainer::GetSplineID(double t_global, const VecSpline& splines)
{
   assert(t_global<=GetTotalTime(splines));

   double t = 0;
   for (const ZmpSpline& s: splines) {
     t += s.GetDuration();

     if (t >= t_global-eps_)
       return s.GetId();
   }
}


double SplineContainer::GetLocalTime(double t_global, const VecSpline& splines)
{
  int id_spline = GetSplineID(t_global,splines);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= splines.at(id).GetDuration();
  }

  return t_local-eps_; // just to never get value greater than true duration due to rounding errors
}


bool SplineContainer::IsFourLegSupport(double t_global) const
{
  assert(t_global<=GetTotalTime());
  return splines_.at(GetSplineID(t_global)).IsFourLegSupport();
}


SplineContainer::Point2d
SplineContainer::GetCOGxy(double t_global, const VecSpline& splines)
{
  int id = GetSplineID(t_global,splines);
  double t_local = GetLocalTime(t_global, splines);

  Point2d cog_xy;
  cog_xy.p = splines[id].GetState(kPos, t_local);
  cog_xy.v = splines[id].GetState(kVel, t_local);
  cog_xy.a = splines[id].GetState(kAcc, t_local);

  return cog_xy;
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
