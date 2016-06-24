/**
@file    spline_container.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Holds coefficients of multiple fifth-order splines and returns state
         (pos,vel,acc) of appropriate spline for a specific time
 */

#include <xpp/zmp/spline_container.h>

namespace xpp {
namespace zmp {

void
SplineContainer::AddSplinesStepSequence (int step_count, double t_swing)
{
  unsigned int id = splines_.size()==0 ? 0 : splines_.back().GetId()+1;

  // fixme more than one currently not supported, change hyq_spliner
  // and ZMP constraint to not relax constraints at these duplicate splines.
  int n_splines_per_step = 1;
  for (int step=0; step<step_count; ++step) {
    for (int i=0; i<n_splines_per_step; ++i) {
      ZmpSpline spline(id++, t_swing/n_splines_per_step, StepSpline);
      spline.SetStep(step);
      splines_.push_back(spline);
    }
  }

  splines_initialized_ = true;
}

void
SplineContainer::AddStanceSpline (double t_stance)
{
  unsigned int id = splines_.size()==0 ? 0 : splines_.back().GetId()+1;
  splines_.push_back(ZmpSpline(id++, t_stance, StanceSpline));

  splines_initialized_ = true;
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
  static constexpr double dt = 0.1; //discretization time [seconds]: needed for creating support triangle inequality constraints

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


