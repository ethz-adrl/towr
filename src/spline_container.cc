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

log4cxx::LoggerPtr SplineContainer::log_(log4cxx::Logger::getLogger("xpp.zmp.splinecontainer"));

SplineContainer::SplineContainer()
{
  splines_.clear();
}


double SplineContainer::GetTotalTime(bool exclude_4ls_splines) const
{
  double T = 0.0;
  for (ZmpSpline s: splines_) {

    if (s.four_leg_supp_ && exclude_4ls_splines)
      continue;

    T += s.duration_;
  };
  return T;
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

  const int kSplinesPer4ls = 1;
  const int kSplinesPerStep = 1;

  for (size_t i = 0; i < step_sequence.size(); ++i)
  {
    // 1. insert 4ls-phase when switching between disjoint support triangles
    if (i==0) {
      AddSpline(ZmpSpline(id++, t_stance_initial, true, step));
    } else {
      xpp::hyq::LegID swing_leg = step_sequence[i];
      xpp::hyq::LegID swing_leg_prev = step_sequence[i-1];
      if (xpp::hyq::SupportPolygonContainer::Insert4LSPhase(swing_leg_prev, swing_leg))
        for (int s = 0; s < kSplinesPer4ls; s++)
          AddSpline(ZmpSpline(id++, t_stance/kSplinesPer4ls, true, step));
    }


    // insert swing phase splines
    for (int s = 0; s < kSplinesPerStep; s++)
      AddSpline(ZmpSpline(id++, t_swing/kSplinesPerStep, false, step));



    // always have last 4ls spline for robot to move into center of feet
    if (i==step_sequence.size()-1)
      AddSpline(ZmpSpline(id++, t_stance_final, true, step));

    step++;
  }
}



void SplineContainer::GetCOGxy(double t_global, Point2d& cog_xy) const
{
  uint curr_spline = 0;
  if (t_global > GetTotalTime())
    throw std::runtime_error("SplineContainer::GetCOGxy: t_global > spline time");

  /** Transform global time to local spline time dt */
  double t_local = t_global;
  while (t_local > splines_[curr_spline].duration_) {
    t_local -= splines_[curr_spline++].duration_;
  }

  cog_xy.p = splines_[curr_spline].GetState(kPos, t_local);
  cog_xy.v = splines_[curr_spline].GetState(kVel, t_local);
  cog_xy.a = splines_[curr_spline].GetState(kAcc, t_local);

  assert(curr_spline < splines_.size()); // make sure the current spline is in the buffer
}

} // namespace zmp
} // namespace xpp
