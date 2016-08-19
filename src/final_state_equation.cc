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

   int c = 0; // constraint count
   for (const Coords3D dim : Coords2DArray)
   {
     ZmpSpline last = splines_.GetLastSpline();
     int K = last.GetId();
     double T = last.GetDuration();

     VecScalar pos = splines_.ExpressComThroughCoeff(kPos, T, K, dim);
     pos.s -= final_state_xy_.p(dim);
     final.WriteRow(pos, c++);

     VecScalar vel = splines_.ExpressComThroughCoeff(kVel, T, K, dim);
     vel.s -= final_state_xy_.v(dim);
     final.WriteRow(vel, c++);

     VecScalar acc = splines_.ExpressComThroughCoeff(kAcc, T, K, dim);
     acc.s -= final_state_xy_.a(dim);
     final.WriteRow(acc, c++);
   }

   assert(c==n_constraints);
   return final;
}

} /* namespace zmp */
} /* namespace xpp */
