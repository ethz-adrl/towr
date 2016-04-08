/*
 * spline_constraints.cc
 *
 *  Created on: Apr 7, 2016
 *      Author: winklera
 */

#include <xpp/zmp/spline_constraints.h>

namespace xpp {
namespace zmp {

SplineConstraints::SplineConstraints (const ContinuousSplineContainer& spline_structure)
    :spline_structure_(spline_structure)
{
  n_opt_coefficients_ = spline_structure_.GetTotalFreeCoeff();
}


SplineConstraints::MatVec
SplineConstraints::CreateInitialAccConstraints(const Vector2d& initial_acc) const
{
  int n_constraints = kDim2d*2; //init {x,y} * {acc, jerk}
  MatVec init(n_constraints, n_opt_coefficients_);

  const Vector2d& initial_jerk = Vector2d::Zero(); // this should never need to be different

  int i = 0; // constraint count
  for (int dim = X; dim <= Y; ++dim)
  {
    // acceleration set to zero
    int d = ContinuousSplineContainer::Index(0, dim, D);
    init.M(i,d) = 2.0;
    init.v(i++) = -initial_acc(dim);
    // jerk set to zero
    int c = ContinuousSplineContainer::Index(0, dim, C);
    init.M(i,c) = 6.0;
    init.v(i++) = -initial_jerk(dim);
  }

  return init;
}


SplineConstraints::MatVec
SplineConstraints::CreateFinalConstraints(const State& final_cond) const
{
  int n_constraints = 3*kDim2d; // pos, vel, acc
  MatVec final(n_constraints, n_opt_coefficients_);

  int i = 0; // constraint count
  for (int dim = X; dim <= Y; ++dim)
  {
    ZmpSpline last = spline_structure_.splines_.back();
    int K = last.id_;
    double T = last.duration_;
    int last_spline = ContinuousSplineContainer::Index(K, dim, A);
    std::array<double,6> t_duration = utils::cache_exponents<6>(T);

    // calculate e and f coefficients from previous values
    VecScalar Ek = spline_structure_.GetCalculatedCoeff(K, dim, E);
    VecScalar Fk = spline_structure_.GetCalculatedCoeff(K, dim, F);

    // position
    final.M(i, last_spline + A) = t_duration[5];
    final.M(i, last_spline + B) = t_duration[4];
    final.M(i, last_spline + C) = t_duration[3];
    final.M(i, last_spline + D) = t_duration[2];
    final.M.row(i) += Ek.v*t_duration[1];
    final.M.row(i) += Fk.v;

    final.v(i)     += Ek.s*t_duration[1] + Fk.s;
    final.v(i++)   += -final_cond.p(dim);

    // velocities
    final.M(i, last_spline + A) = 5 * t_duration[4];
    final.M(i, last_spline + B) = 4 * t_duration[3];
    final.M(i, last_spline + C) = 3 * t_duration[2];
    final.M(i, last_spline + D) = 2 * t_duration[1];
    final.M.row(i) += Ek.v;

    final.v(i)     += Ek.s;
    final.v(i++)   += -final_cond.v(dim);

    // accelerations
    final.M(i, last_spline + A) = 20 * t_duration[3];
    final.M(i, last_spline + B) = 12 * t_duration[2];
    final.M(i, last_spline + C) = 6  * t_duration[1];
    final.M(i, last_spline + D) = 2;

    final.v(i++) = -final_cond.a(dim);
  }

  return final;
}


SplineConstraints::MatVec
SplineConstraints::CreateJunctionConstraints() const
{
  // junctions {acc,jerk} since pos, vel  implied
  int n_constraints = (spline_structure_.splines_.size()-1) * kDim2d * 2;
  MatVec junction(n_constraints, n_opt_coefficients_);

  // FIXME maybe replace with range based loop
  int i = 0; // constraint count
  for (uint s = 0; s < spline_structure_.splines_.size() - 1; ++s)
  {
    double T = spline_structure_.splines_.at(s).duration_;
    std::array<double,6> t_duration = utils::cache_exponents<6>(T);
    for (int dim = X; dim <= Y; dim++) {

      int curr_spline = ContinuousSplineContainer::Index(s, dim, A);
      int next_spline = ContinuousSplineContainer::Index(s + 1, dim, A);

      // acceleration
      junction.M(i, curr_spline + A) = 20 * t_duration[3];
      junction.M(i, curr_spline + B) = 12 * t_duration[2];
      junction.M(i, curr_spline + C) = 6  * t_duration[1];
      junction.M(i, curr_spline + D) = 2;
      junction.M(i, next_spline + D) = -2.0;
      junction.v(i++) = 0.0;

      // jerk (derivative of acceleration)
      junction.M(i, curr_spline + A) = 60 * t_duration[2];
      junction.M(i, curr_spline + B) = 24 * t_duration[1];
      junction.M(i, curr_spline + C) = 6;
      junction.M(i, next_spline + C) = -6.0;
      junction.v(i++) = 0.0;
    }
  }
  return junction;
}


SplineConstraints::MatVec
SplineConstraints::CreateAllSplineConstraints(const State &final_state) const
{
  MatVec spline_constraints;
  spline_constraints << CreateInitialAccConstraints();
  spline_constraints << CreateFinalConstraints(final_state);
  spline_constraints << CreateJunctionConstraints();

  return spline_constraints;
}

} /* namespace zmp */
} /* namespace xpp */
