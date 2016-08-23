/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include "../include/xpp/zmp/com_polynomial_fifth_order.h"

namespace xpp {
namespace zmp {

ComSpline::ComSpline ()
{
  // TODO Auto-generated constructor stub
}

ComSpline::~ComSpline ()
{
  // TODO Auto-generated destructor stub
}

void
ComSpline::AddSplinesStepSequence (int step_count, double t_swing)
{
  unsigned int id = splines_.size()==0 ? 0 : splines_.back().GetId()+1;

  // fixme more than one currently not supported, change hyq_spliner
  // and ZMP constraint to not relax constraints at these duplicate splines.
  int n_splines_per_step = 1;
  for (int step=0; step<step_count; ++step) {
    for (int i=0; i<n_splines_per_step; ++i) {
      ComPolynomial spline(id++, t_swing/n_splines_per_step, StepSpline);
      spline.SetStep(step);
      splines_.push_back(spline);
    }
  }

  splines_initialized_ = true;
}

void
ComSpline::AddStanceSpline (double t_stance)
{
  unsigned int id = splines_.size()==0 ? 0 : splines_.back().GetId()+1;
  splines_.push_back(ComPolynomial(id++, t_stance, StanceSpline));

  splines_initialized_ = true;
}


double ComSpline::GetTotalTime(const VecSpline& splines)
{
  double T = 0.0;
  for (const ComPolynomial& s: splines)
    T += s.GetDuration();
  return T;
}


int ComSpline::GetSplineID(double t_global, const VecSpline& splines)
{
   assert(t_global<=GetTotalTime(splines));

   double t = 0;
   for (const ComPolynomial& s: splines) {
     t += s.GetDuration();

     if (t >= t_global) // at junctions, returns previous spline (=)
       return s.GetId();
   }
   assert(false); // this should never be reached
}


std::vector<double>
ComSpline::GetDiscretizedGlobalTimes(const VecSpline& splines)
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


double ComSpline::GetLocalTime(double t_global, const VecSpline& splines)
{
  int id_spline = GetSplineID(t_global,splines);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= splines.at(id).GetDuration();
  }

  return t_local;//-eps_; // just to never get value greater than true duration due to rounding errors
}


ComSpline::Point2d
ComSpline::GetCOGxy(double t_global, const VecSpline& splines)
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
ComSpline::CheckIfSplinesInitialized() const
{
  if (!splines_initialized_) {
    throw std::runtime_error("ComSpline.splines_ not initialized. Call Init() first");
  }
}



} /* namespace zmp */
} /* namespace xpp */
