/*
 * final_state_constraint.cc
 *
 *  Created on: May 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/final_state_constraint.h>

namespace xpp {
namespace zmp {

FinalStateConstraint::FinalStateConstraint (OptimizationVariables& subject)
{
  subject_ = &subject;
  subject_->RegisterObserver(this);
}

void
FinalStateConstraint::Init (const State2d& final_xy,
                            const ContinuousSplineContainer& c)
{
  lin_constraint_ = BuildLinearConstraint(final_xy, c);
  Update();
}

void
FinalStateConstraint::Update ()
{
  x_coeff_ = subject_->GetSplineCoefficients();
}

FinalStateConstraint::VectorXd
FinalStateConstraint::EvaluateConstraint () const
{
  return lin_constraint_.M*x_coeff_ + lin_constraint_.v;
}

FinalStateConstraint::VecBound
FinalStateConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  VectorXd g = EvaluateConstraint();

  for (int i=0; i<g.rows(); ++i)
    bounds.push_back(kEqualityBound_);

  return bounds;
}

FinalStateConstraint::MatVec
FinalStateConstraint::BuildLinearConstraint(const State2d& final_xy,
                                            const ContinuousSplineContainer& spline_container)
{
  using namespace xpp::utils;

  int n_constraints = 3*kDim2d; // pos, vel, acc
  int n_spline_coeff = subject_->GetSplineCoefficients().rows();
  MatVec final(n_constraints, n_spline_coeff);

  int i = 0; // constraint count
  for (const Coords3D dim : Coords2DArray)
  {
    ZmpSpline last = spline_container.GetLastSpline();
    int K = last.GetId();
    double T = last.GetDuration();
    int last_spline = ContinuousSplineContainer::Index(K, dim, A);
    std::array<double,6> t_duration = utils::cache_exponents<6>(T);

    // calculate e and f coefficients from previous values
    VecScalar Ek = spline_container.GetECoefficient(K, dim);
    VecScalar Fk = spline_container.GetFCoefficient(K, dim);

    // position
    final.M(i, last_spline + A) = t_duration[5];
    final.M(i, last_spline + B) = t_duration[4];
    final.M(i, last_spline + C) = t_duration[3];
    final.M(i, last_spline + D) = t_duration[2];
    final.M.row(i) += Ek.v*t_duration[1];
    final.M.row(i) += Fk.v;

    final.v(i)     += Ek.s*t_duration[1] + Fk.s;
    final.v(i++)   += -final_xy.p(dim);

    // velocities
    final.M(i, last_spline + A) = 5 * t_duration[4];
    final.M(i, last_spline + B) = 4 * t_duration[3];
    final.M(i, last_spline + C) = 3 * t_duration[2];
    final.M(i, last_spline + D) = 2 * t_duration[1];
    final.M.row(i) += Ek.v;

    final.v(i)     += Ek.s;
    final.v(i++)   += -final_xy.v(dim);

    // accelerations
    final.M(i, last_spline + A) = 20 * t_duration[3];
    final.M(i, last_spline + B) = 12 * t_duration[2];
    final.M(i, last_spline + C) = 6  * t_duration[1];
    final.M(i, last_spline + D) = 2;

    final.v(i++) = -final_xy.a(dim);
  }

  assert(i==n_constraints);
  return final;
}



} /* namespace zmp */
} /* namespace xpp */
