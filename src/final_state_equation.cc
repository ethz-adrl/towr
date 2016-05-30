/**
 @file    final_state_equation.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/zmp/final_state_equation.h>

namespace xpp {
namespace zmp {

FinalStateEquation::FinalStateEquation (const State2d& final_state_xy,
                                        const ContinuousSplineContainer& splines)
    :final_state_xy_(final_state_xy),
     splines_(splines)
{}

FinalStateEquation::MatVec
FinalStateEquation::BuildLinearEquation () const
{
  using namespace xpp::utils;

   int n_constraints = 3*kDim2d; // pos, vel, acc
   int n_spline_coeff = splines_.GetTotalFreeCoeff();
   MatVec final(n_constraints, n_spline_coeff);

   int i = 0; // constraint count
   for (const Coords3D dim : Coords2DArray)
   {
     ZmpSpline last = splines_.GetLastSpline();
     int K = last.GetId();
     double T = last.GetDuration();
     int last_spline = ContinuousSplineContainer::Index(K, dim, A);
     std::array<double,6> t_duration = utils::cache_exponents<6>(T);

     // calculate e and f coefficients from previous values
     VecScalar Ek = splines_.GetECoefficient(K, dim);
     VecScalar Fk = splines_.GetFCoefficient(K, dim);

     // position
     final.M(i, last_spline + A) = t_duration[5];
     final.M(i, last_spline + B) = t_duration[4];
     final.M(i, last_spline + C) = t_duration[3];
     final.M(i, last_spline + D) = t_duration[2];
     final.M.row(i) += Ek.v*t_duration[1];
     final.M.row(i) += Fk.v;

     final.v(i)     += Ek.s*t_duration[1] + Fk.s;
     final.v(i++)   += -final_state_xy_.p(dim);

     // velocities
     final.M(i, last_spline + A) = 5 * t_duration[4];
     final.M(i, last_spline + B) = 4 * t_duration[3];
     final.M(i, last_spline + C) = 3 * t_duration[2];
     final.M(i, last_spline + D) = 2 * t_duration[1];
     final.M.row(i) += Ek.v;

     final.v(i)     += Ek.s;
     final.v(i++)   += -final_state_xy_.v(dim);

     // accelerations
     final.M(i, last_spline + A) = 20 * t_duration[3];
     final.M(i, last_spline + B) = 12 * t_duration[2];
     final.M(i, last_spline + C) = 6  * t_duration[1];
     final.M(i, last_spline + D) = 2;

     final.v(i++) = -final_state_xy_.a(dim);
   }

   assert(i==n_constraints);
   return final;

}

} /* namespace zmp */
} /* namespace xpp */
