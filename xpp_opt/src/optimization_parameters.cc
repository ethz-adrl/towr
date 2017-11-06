/**
@file    motion_type.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#include <../include/xpp_opt/optimization_parameters.h>

#include <algorithm>
#include <initializer_list>
#include <iterator>

namespace xpp {

OptimizationParameters::OptimizationParameters ()
{
  order_coeff_polys_  = 4; // used only with coeff_spline representation
  dt_base_polynomial_ = 0.2;


  // 2 also works quite well. Remember that inbetween the nodes, forces
  // could still be violating unilateral and friction constraints by
  // polynomial interpolation
  force_splines_per_stance_phase_ = 3; // this makes it a lot bigger


  // range of motion constraint
  dt_range_of_motion_ = 0.1;
  ee_splines_per_swing_phase_ = 2; // should always be 2 if i want to use swing constraint!

  dt_base_range_of_motion_ = dt_base_polynomial_/4.;


  min_phase_duration_ = 0.1;
  double max_time = 1.0; // this helps convergence
  max_phase_duration_ = max_time>GetTotalTime()?  GetTotalTime() : max_time;
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();

  constraints_ = {
      BasePoly, // include this results in non-hermite representation to be used
//      BaseRom, //  CAREFUL: restricts the base to be in a specific range->very limiting
      EndeffectorRom,
      Dynamic,
      Terrain,
      Force,
//      TotalTime, // Attention: this causes segfault in SNOPT
      Swing, // this is important for lifting leg
  };

  cost_weights_ = {
//      {ForcesCostID, 1.0},
//      {ComCostID, 1.0}
  };
}

OptimizationParameters::CostWeights
OptimizationParameters::GetCostWeights () const
{
  return cost_weights_;
}

OptimizationParameters::UsedConstraints
OptimizationParameters::GetUsedConstraints () const
{
  return constraints_;
}

bool
OptimizationParameters::ConstraintExists (ConstraintName c) const
{
  auto v = constraints_; // shorthand
  return std::find(v.begin(), v.end(), c) != v.end();
}

OptimizationParameters::VecTimes
OptimizationParameters::GetBasePolyDurations () const
{
  std::vector<double> base_spline_timings_;
  double dt = dt_base_polynomial_;
  double t_left = t_total_;

  double eps = 1e-10; // since repeated subtraction causes inaccuracies
  while (t_left > eps) {
    double duration = t_left>dt?  dt : t_left;
    base_spline_timings_.push_back(duration);

    t_left -= dt;
  }

  return base_spline_timings_;
}

OptimizationParameters::BaseRepresentation
OptimizationParameters::GetBaseRepresentation () const
{
  auto v = constraints_; // alias
  if(std::find(v.begin(), v.end(), BasePoly) != v.end()) {
      return PolyCoeff; // v contains element
  } else {
     return CubicHermite;
  }
}

OptimizationParameters::~OptimizationParameters ()
{
}

} // namespace xpp

