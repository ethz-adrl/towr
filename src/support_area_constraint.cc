/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Defines the DynamicConstraint class
 */

#include <xpp/opt/support_area_constraint.h>

namespace xpp {
namespace opt {

SupportAreaConstraint::SupportAreaConstraint ()
{
  name_ = "Support Area";
}

SupportAreaConstraint::~SupportAreaConstraint ()
{
}

void
SupportAreaConstraint::Init (const EEMotionPtr& ee_motion,
                             const EELoadPtr& ee_load,
                             const CopPtr& cop,
                             double T,
                             double dt)
{
  ee_motion_ = ee_motion;
  ee_load_ = ee_load;
  cop_ = cop;

  double t = 0.0;
  dts_.clear();
  for (int i=0; i<floor(T/dt); ++i) {
    dts_.push_back(t);
    t += dt;
  }

  int num_constraints = dts_.size()*kDim2d;
  SetDependentVariables({ee_motion, ee_load, cop}, num_constraints);

  UpdateJacobianWithRespectToCop(); // constant, e.g. not depdent on opt. values
}

void
SupportAreaConstraint::UpdateConstraintValues ()
{
  int k = 0;
  for (double t : dts_) {

    Vector2d convex_contacts = Vector2d::Zero();
    auto lambda_k = ee_load_->GetLoadValues(t);

    // spring_clean_ could actually also be all the endeffectors, then contact flags would only
    // be in other constraint
    for (auto f : ee_motion_->GetContacts(t))
      convex_contacts += lambda_k.At(f.ee)*f.p.topRows<kDim2d>();

    Vector2d cop = cop_->GetCop(t);
    g_.middleRows<kDim2d>(kDim2d*k) = convex_contacts - cop;
    k++;
  }
}

void
SupportAreaConstraint::UpdateBounds ()
{
  std::fill(bounds_.begin(), bounds_.end(), kEqualityBound_);
}

void
SupportAreaConstraint::UpdateJacobianWithRespectToLoad()
{
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_load_->GetID());

  int row_idx = 0;
  for (double t : dts_) {

    for (auto f : ee_motion_->GetContacts(t)) {
      for (auto dim : d2::AllDimensions) {
        int idx = ee_load_->Index(t,f.ee);
        jac.coeffRef(row_idx+dim,idx) = f.p(dim);
      }
    }
    row_idx += kDim2d;
  }
}

void
SupportAreaConstraint::UpdateJacobianWithRespectToEEMotion ()
{
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_motion_->GetID());

  int row_idx = 0;
  for (double t : dts_) {

    auto lambda_k = ee_load_->GetLoadValues(t);
    for (auto f : ee_motion_->GetContacts(t)) {
      if (f.id != ContactBase::kFixedByStartStance) {
        for (auto dim : d2::AllDimensions) {
          int idx_contact = ee_motion_->Index(f.ee, f.id, dim);
          jac.coeffRef(row_idx+dim, idx_contact) = lambda_k.At(f.ee);
        }
      }
    }

    row_idx += kDim2d;
  }
}

void
SupportAreaConstraint::UpdateJacobianWithRespectToCop ()
{
  Jacobian& jac = GetJacobianRefWithRespectTo(cop_->GetID());

  int row = 0;
  for (double t : dts_)
    for (auto dim : d2::AllDimensions)
      jac.row(row++) = -1 * cop_->GetJacobianWrtCop(t,dim);
}

void
SupportAreaConstraint::UpdateJacobians ()
{
  UpdateJacobianWithRespectToLoad();
  UpdateJacobianWithRespectToEEMotion();
//  UpdateJacobianWithRespectToCop(); // not dependent on opt. values
}


} /* namespace opt */
} /* namespace xpp */

