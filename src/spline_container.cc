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
                                 const SplineTimes& times)
{
  Init(step_sequence, times);
}


void SplineContainer::Init(const std::vector<xpp::hyq::LegID>& step_sequence,
                           const SplineTimes& times)
{
  splines_ = ConstructSplineSequence(step_sequence, times);
  splines_initialized_ = true;
}


// Creates a sequence of Splines without the optimized coefficients
// fixme this function is definetly doing more than one thing -> split up
SplineContainer::VecSpline
SplineContainer::ConstructSplineSequence(const std::vector<LegID>& step_sequence,
                                         const SplineTimes& times)
{
  VecSpline splines;
  int step = 0;
  unsigned int id = 0; // unique identifiers for each spline

  const double t_init_max = 0.4; //s
  int n_init_splines = std::ceil(times.t_stance_initial_/t_init_max);

  double t_init_spline = times.t_stance_initial_/n_init_splines;

  for (int i=0; i<n_init_splines; ++i)
    splines.push_back(ZmpSpline(id++, t_init_spline, Initial4lsSpline, step));


  // just body shift
  if (step_sequence.size() == 0)
    return splines;


  splines.push_back(ZmpSpline(id++, times.t_swing_, StepSpline, step));
  for (int i=1; i<step_sequence.size(); ++i)
  {
    step++;

//    // 1. insert 4ls-phase when switching between disjoint support triangles
//    xpp::hyq::LegID swing_leg = step_sequence[i];
//    xpp::hyq::LegID swing_leg_prev = step_sequence[i-1];
//    if (xpp::hyq::SupportPolygonContainer::Insert4LSPhase(swing_leg_prev, swing_leg))
//      splines.push_back(ZmpSpline(id++, times.t_stance_, Intermediate4lsSpline, step));

    // insert swing phase splines
    splines.push_back(ZmpSpline(id++, times.t_swing_, StepSpline, step));
  }


//  const double t_final_max = 0.4; //s
//  int n_final_splines = std::ceil(times.t_stance_final_/t_final_max);
//
//  // always have last 4ls spline for robot to move into center of feet
//  const uint NO_NEXT_STEP = std::numeric_limits<uint>::max();
//  for (int i=0; i<n_final_splines; ++i)
//    splines.push_back(ZmpSpline(id++, times.t_stance_final_/n_final_splines, Final4lsSpline, NO_NEXT_STEP));

  return splines;
}

SplineContainer::VecSpline
SplineContainer::ConstructSplineSequenceBare (const VecLegID& step_sequence,
                                              const SplineTimes& times)
{
  VecSpline splines;
  int step = 0;
  unsigned int id = 0; // unique identifiers for each spline

  splines.push_back(ZmpSpline(id++, times.t_swing_, StepSpline, step));
  for (int i=1; i<step_sequence.size(); ++i)
  {
    step++;

    // 1. insert 4ls-phase when switching between disjoint support triangles
    xpp::hyq::LegID swing_leg = step_sequence[i];
    xpp::hyq::LegID swing_leg_prev = step_sequence[i-1];
    if (xpp::hyq::SupportPolygonContainer::Insert4LSPhase(swing_leg_prev, swing_leg))
      splines.push_back(ZmpSpline(id++, times.t_stance_, Intermediate4lsSpline, step));

    // insert swing phase splines
    splines.push_back(ZmpSpline(id++, times.t_swing_, StepSpline, step));
  }

  return splines;
}


double SplineContainer::GetTotalTime(const VecSpline& splines)
{
  double T = 0.0;
  for (const ZmpSpline& s: splines)
    T += s.GetDuration();
  return T;
}


int SplineContainer::GetSplineID(double t_global, const VecSpline& splines)
{
   assert(t_global<=GetTotalTime(splines));

   double t = 0;
   for (const ZmpSpline& s: splines) {
     t += s.GetDuration();

     if (t >= t_global) // at junctions, returns previous spline (=)
       return s.GetId();
   }
   assert(false); // this should never be reached
}


std::vector<double>
SplineContainer::GetDiscretizedGlobalTimes(const VecSpline& splines)
{
  static constexpr double dt = 0.2; //discretization time [seconds]: needed for creating support triangle inequality constraints

  std::vector<double> vec;
  double t = 0.0;
  while (t <= GetTotalTime(splines)-dt+eps_) { // still add the second to last time, even if rounding errors to to floating point arithmetics
    vec.push_back(t);
    t += dt;
  }

  vec.push_back(GetTotalTime(splines));
  return vec;
}


double SplineContainer::GetLocalTime(double t_global, const VecSpline& splines)
{
  int id_spline = GetSplineID(t_global,splines);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= splines.at(id).GetDuration();
  }

  return t_local;//-eps_; // just to never get value greater than true duration due to rounding errors
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
  cog_xy.p = splines[id].GetState(xpp::utils::kPos, t_local);
  cog_xy.v = splines[id].GetState(xpp::utils::kVel, t_local);
  cog_xy.a = splines[id].GetState(xpp::utils::kAcc, t_local);

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


