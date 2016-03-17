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
//  T = 0.0;
  curr_spline_ = 0;
  splines_.clear();
}


SplineContainer::~SplineContainer()
{
  // TODO Auto-generated destructor stub
}

double SplineContainer::GetTotalTime() const
{
  double T = 0.0;
  for (ZmpSpline s: splines_) {
    T += s.duration_;
  };
  return T;
}

void SplineContainer::AddSpline(const ZmpSpline &spline)
{
  splines_.push_back(spline);
//  T += spline.duration_;
}



void SplineContainer::AddSplines(const Splines &splines)
{
  curr_spline_ = 0;
  splines_ = splines;
//  for (ZmpSpline s: splines) {
//    T += s.duration_;
//  }
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
