/**
@file    spline_container.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Holds coefficients of multiple fifth-order splines and returns state
         (pos,vel,acc) of appropriate spline for a specific time
 */

#include <xpp/zmp/spline_container.h>

#include <cmath>
#include <iostream>
#include <iomanip>      // std::setprecision
#include <cassert>

namespace xpp {
namespace zmp {

log4cxx::LoggerPtr SplineContainer::log_(log4cxx::Logger::getLogger("xpp.zmp.splinecontainer"));

SplineContainer::SplineContainer()
{
  curr_spline_ = 0;
  splines_.clear();
}


SplineContainer::~SplineContainer()
{
  // TODO Auto-generated destructor stub
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


int SplineContainer::GetTotalNodes(double dt, bool exclude_4ls_splines) const
{
  int node_count = 0;

  for (ZmpSpline s: splines_) {

    if (s.four_leg_supp_ && exclude_4ls_splines)
      continue;

    node_count += s.GetNodeCount(dt);
  };
  return node_count;
}

// TODO create lookup table for this
int SplineContainer::GetSplineID(int node, double dt) const
{
  int n = 0;
  for (ZmpSpline s: splines_) {
    n += s.GetNodeCount(dt);

    if (n > node)
      return s.id_;
  }
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
    // Attention: these 4ls-phases much coincide with the ones in the zmp optimizer
    if (i==0) {
      AddSpline(ZmpSpline(id++, t_stance_initial, true, step));
    } else {
      xpp::hyq::LegID swing_leg = step_sequence[i];
      xpp::hyq::LegID swing_leg_prev = step_sequence[i-1];
      if (Insert4LSPhase(swing_leg_prev, swing_leg))
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


bool SplineContainer::Insert4LSPhase(LegID prev, LegID next)
{
  using namespace xpp::hyq;
  // check for switching between disjoint support triangles.
  // the direction the robot is moving between triangles does not matter.
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;
  std::swap(prev, next);
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;

  return false;
}



void SplineContainer::GetCOGxy(double t_global, Lin2d& cog_xy)
{
  /** Transform global time to local spline time dt */
  double t_local = t_global;
  for (uint s = 0; s < curr_spline_; s++)
    t_local -= splines_[s].duration_;

  cog_xy.p = splines_[curr_spline_].GetState(kPos, t_local);
  cog_xy.v = splines_[curr_spline_].GetState(kVel, t_local);
  cog_xy.a = splines_[curr_spline_].GetState(kAcc, t_local);

  if (t_local > splines_[curr_spline_].duration_) {
    ++curr_spline_;
    assert(curr_spline_ < splines_.size()); // make sure the current spline is in the buffer
    LOG4CXX_TRACE(log_, std::setprecision(1) << std::fixed << " switched to zmp-spline " << curr_spline_);
  }
}

} // namespace zmp
} // namespace xpp
