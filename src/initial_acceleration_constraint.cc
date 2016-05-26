/*
 * initial_acceleration_constraint.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/initial_acceleration_constraint.h>

#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace zmp {

InitialAccelerationConstraint::InitialAccelerationConstraint (
  OptimizationVariables& subject)
{
  subject_ = &subject;
  subject_->RegisterObserver(this);

  Update();
}

void
InitialAccelerationConstraint::Init (
    const Vector2d& acc_xy)
{
  lin_constraint_ = BuildLinearConstraint(acc_xy);
}

void
InitialAccelerationConstraint::Update ()
{
  x_coeff_ = subject_->GetSplineCoefficients();
}

InitialAccelerationConstraint::VectorXd
InitialAccelerationConstraint::EvaluateConstraint () const
{
  return lin_constraint_.M*x_coeff_ + lin_constraint_.v;
}

InitialAccelerationConstraint::VecBound
InitialAccelerationConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  VectorXd g = EvaluateConstraint();

  for (int i=0; i<g.rows(); ++i)
    bounds.push_back(kEqualityBound_);

  return bounds;
}

InitialAccelerationConstraint::MatVec
InitialAccelerationConstraint::BuildLinearConstraint (const Vector2d& initial_acc)
{
  using namespace xpp::utils;

  int n_constraints = kDim2d *1; // acceleration in x and y direction
  int n_spline_coeff = subject_->GetSplineCoefficients().rows();
  MatVec lin_constraint(n_constraints, n_spline_coeff);

  int i = 0; // constraint count
  for (const Coords3D dim : Coords2DArray)
  {
    // acceleration set to zero
    int d = ContinuousSplineContainer::Index(0, dim, D);
    lin_constraint.M(i,d) = 2.0;
    lin_constraint.v(i++) = -initial_acc(dim);
  }

  assert(i==n_constraints);
  return lin_constraint;
}

} /* namespace zmp */
} /* namespace xpp */


