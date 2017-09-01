/**
@file    motion_type.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#include <xpp/optimization_parameters.h>

#include <algorithm>
#include <iterator>

#include <xpp/cartesian_declarations.h>

namespace xpp {
namespace opt {

//MotionParameters::ContactSchedule
//MotionParameters::GetContactSchedule () const
//{
//  ContactSchedule phases;
//  for (int i=0; i<contact_sequence_.size(); ++i) {
//    double duration = contact_timings_.at(i);
////    if (duration < 1e-10)
////      continue; // skip phases with zero duration
//    phases.push_back(Phase(contact_sequence_.at(i), duration));
//  }
//
//  return phases;
//}

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

double
OptimizationParameters::GetTotalTime () const
{
  double T = 0.0;
  for (auto t : contact_timings_.at(E0)) // use leg 0 for calculation
    T += t;

  return T;
}

OptimizationParameters::VecTimes
OptimizationParameters::GetBasePolyDurations () const
{
  std::vector<double> base_spline_timings_;
  double dt = dt_base_polynomial_;
  double t_left = GetTotalTime();

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




// some specific implementations
MonopedOptParameters::MonopedOptParameters()
{
  order_coeff_polys_ = 4;
  dt_base_polynomial_ = 0.1;

  force_splines_per_stance_phase_ = 3;

  dt_range_of_motion_ = 0.1;
  ee_splines_per_swing_phase_ = 1;

  double f  = 0.2;
  double c = 0.2;
  contact_timings_ = ContactTimings(1);
  contact_timings_.at(E0) = {c, f, c, f, c, f, c, f, c, c, f, c, f, c, f, c, f, c};

  min_phase_duration_ = 0.1;
  max_phase_duration_ = GetTotalTime();
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();


  constraints_ = {
      BasePoly,
      RomBox,
      Dynamic,
      Terrain,
      TotalTime,
      Swing
  };

}

BipedOptParameters::BipedOptParameters()
{
  order_coeff_polys_ = 4;
  dt_base_polynomial_ = 0.1;

  ee_splines_per_swing_phase_ = 1;
  force_splines_per_stance_phase_ = 3;


  dt_range_of_motion_ = 0.1;


  double f  = 0.2;
  double c = 0.2;
  double offset = c;
  contact_timings_ = ContactTimings(2);
  contact_timings_.at(E0) = {c+offset,f,c,f,c,f,c,f,c,f,c,f,c,f, c};
  contact_timings_.at(E1) = {       c,f,c,f,c,f,c,f,c,f,c,f,c,f, c+offset};


  min_phase_duration_ = 0.1;
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();
  max_phase_duration_ = GetTotalTime();


  constraints_ = {
      BasePoly,
      RomBox,
      Dynamic,
      Terrain,
      TotalTime,
      Swing,
  };
}



QuadrupedOptParameters::QuadrupedOptParameters ()
{
  using namespace xpp::quad;

  order_coeff_polys_  = 4; // used only with coeff_spline representation
  dt_base_polynomial_ = 0.2;


  force_splines_per_stance_phase_ = 3;


  // range of motion constraint
  dt_range_of_motion_ = 0.15;
  // not used, hardcoded for xy and z.
  ee_splines_per_swing_phase_ = 1; // should always be 2 if i want to use swing constraint!



  double f = 0.25; // [s] t_free
  double c = 0.25; // [s] t_contact
  double t_offset = f;
  contact_timings_ = ContactTimings(4);
  auto m = Reverse(kMapOptToQuad);
  contact_timings_.at(m.at(LH)) = {t_offset + c, f, c, f, c, f, c, f, c           };
  contact_timings_.at(m.at(LF)) = {           c, f, c, f, c, f, c, f, c + t_offset};
  contact_timings_.at(m.at(RH)) = {           c, f, c, f, c, f, c, f, c + t_offset};
  contact_timings_.at(m.at(RF)) = {t_offset + c, f, c, f, c, f, c, f, c           };


  min_phase_duration_ = 0.05;
  double max_time = 10.0;
  max_phase_duration_ = max_time>GetTotalTime()?  GetTotalTime() : max_time;
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();

  constraints_ = {
      BasePoly, // include this results in non-hermite representation to be used
      RomBox,
      Dynamic,
      Terrain,
      Force,
      TotalTime, // Attention: this causes segfault in SNOPT
//      Swing,
  };

  cost_weights_ = {
//      {ForcesCostID, 1.0},
//      {ComCostID, 1.0}
  };
}


QuadrotorOptParameters::QuadrotorOptParameters ()
{
  using namespace xpp::quad;

  order_coeff_polys_  = 4;
  dt_base_polynomial_ = 0.2; //s 0.05


  force_splines_per_stance_phase_ = 3;


  // range of motion constraint
  dt_range_of_motion_ = 0.1;
  ee_splines_per_swing_phase_ = 1;



  double f = 0.2; // [s] t_free
  double c = 0.2; // [s] t_contact
  double t_offset = f;
  contact_timings_ = ContactTimings(4);
  auto m = Reverse(kMapOptToRotor);
  contact_timings_.at(m.at(L)) = {t_offset + c, f, c, f, c, f, c           };
  contact_timings_.at(m.at(R)) = {           c, f, c, f, c, f, c + t_offset};
  contact_timings_.at(m.at(F)) = {           c, f, c, f, c, f, c + t_offset};
  contact_timings_.at(m.at(H)) = {t_offset + c, f, c, f, c, f, c           };

  min_phase_duration_ = 0.1;
  max_phase_duration_ = GetTotalTime();
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();

  constraints_ = {
      //BasePoly
      RomBox,
      Dynamic,
      Swing,
//      TotalTime,
  };

  cost_weights_ = {
//      {ForcesCostID, 1.0},
//      {ComCostID, 1.0}
  };
}





} // namespace opt
} // namespace xpp

