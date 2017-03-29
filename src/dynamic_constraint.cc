/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/dynamic_constraint.h>

#include "../include/xpp/opt/base_motion.h"

namespace xpp {
namespace opt {

using Vector2d = Eigen::Vector2d;
using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;

DynamicConstraint::DynamicConstraint ()
{
  kHeight_ = 0.0;
  name_ = "Dynamic";
}

DynamicConstraint::~DynamicConstraint ()
{
}

void
DynamicConstraint::Init (const BaseMotionPtr& com_motion,
                         const CopPtr& cop,
                         double T,
                         double dt)
{
  com_motion_ = com_motion;
  kHeight_ = com_motion->GetZHeight();
  cop_ = cop;

  // zmp_ DRY with other constraints?
  double t = 0.0;
  dts_.clear();
  for (int i=0; i<floor(T/dt); ++i) {
    dts_.push_back(t);
    t += dt;
  }

  int num_constraints = dts_.size()*kDim2d;
  SetDependentVariables({com_motion, cop}, num_constraints);
}

void
DynamicConstraint::UpdateConstraintValues ()
{
  int k = 0;
  for (double t : dts_) {

    auto com = com_motion_->GetCom(t);
    model_.SetCurrent(com.p, com.v, kHeight_);

    // acceleration as predefined by physics
    Vector2d acc_physics = model_.GetDerivative(cop_->GetCop(t));
    g_.middleRows<kDim2d>(kDim2d*k) = acc_physics - com.a;

    k++;
  }
}

void
DynamicConstraint::UpdateBounds ()
{
  std::fill(bounds_.begin(), bounds_.end(), kEqualityBound_);
}

void
DynamicConstraint::UpdateJacobianWrtCop ()
{
  Jacobian& jac = GetJacobianRefWithRespectTo(cop_->GetID());

  int row=0;
  for (double t : dts_) {

    auto com = com_motion_->GetCom(t);
    model_.SetCurrent(com.p, com.v, kHeight_);

    for (auto dim : d2::AllDimensions)
      jac.row(row++) = model_.GetJacobianApproxWrtCop(dim)*cop_->GetJacobianWrtCop(t,dim);
  }
}

void
DynamicConstraint::UpdateJacobians ()
{
  UpdateJacobianWrtCop();
  UpdateJacobianWrtCom();
}

void
DynamicConstraint::UpdateJacobianWrtCom ()
{
  Jacobian& jac = GetJacobianRefWithRespectTo(com_motion_->GetID());

  int n=0;
  for (double t : dts_) {

    auto com = com_motion_->GetCom(t);
    model_.SetCurrent(com.p, com.v, kHeight_);
    Vector2d cop = cop_->GetCop(t);

    for (auto dim : {X, Y}) {
      Jacobian jac_acc     = com_motion_->GetJacobian(t,kAcc,dim);
      Jacobian jac_physics = model_.GetJacobianApproxWrtSplineCoeff(*com_motion_, t, dim, cop);
      jac.row(kDim2d*n + dim) = jac_physics - jac_acc;
    }

    n++;
  }
}


} /* namespace opt */
} /* namespace xpp */
