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
  // TODO Auto-generated destructor stub
}

void
SupportAreaConstraint::Init (const EndeffectorsMotion& ee_motion,
                             const EndeffectorLoad& ee_load,
                             const CenterOfPressure& cop,
                             double T,
                             double dt)
{
  ee_motion_ = ee_motion;
  ee_load_ = ee_load;
  cop_ = cop;

  double t = 0;
  dts_.clear();
  for (int i=0; i<floor(T/dt); ++i) {
    dts_.push_back(t);
    t += dt;
  }
}

void
SupportAreaConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd lambdas   = opt_var->GetVariables(EndeffectorLoad::ID);
  VectorXd footholds = opt_var->GetVariables(EndeffectorsMotion::ID);
  VectorXd cop = opt_var->GetVariables(CenterOfPressure::ID);

  ee_motion_.SetOptimizationParameters(footholds);
  ee_load_.SetOptimizationVariables(lambdas);
  cop_.SetOptimizationVariables(cop);
}

SupportAreaConstraint::VectorXd
SupportAreaConstraint::EvaluateConstraint () const
{
//  int m = motion_structure_.GetPhaseStampedVec().size() * kDim2d;
  int m = dts_.size() * kDim2d;
  Eigen::VectorXd g(m);

  int idx_lambda = 0;
  int k = 0;
  for (double t : dts_) {
//  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {

    Vector2d convex_contacts;
    convex_contacts.setZero();

    for (auto f : ee_motion_.GetContacts(t)) {
      double lamdba = ee_load_.GetOptimizationVariables()(idx_lambda++);
      convex_contacts += lamdba*f.p.topRows<kDim2d>();
    }
//    for (auto f : node.GetAllContacts(footholds_)) {
//      double lamdba = lambdas_(idx_lambda++);
//      convex_contacts += lamdba*f.p.topRows<kDim2d>();
//    }

    g.middleRows<kDim2d>(kDim2d*k) = convex_contacts - cop_.GetOptimizationVariables().middleRows<kDim2d>(kDim2d*k);
    k++;
  }

  return g;
}

VecBound
SupportAreaConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  int m = EvaluateConstraint().rows();
  for (int i=0; i<m; ++i)
    bounds.push_back(kEqualityBound_);

  return bounds;
}

SupportAreaConstraint::Jacobian
SupportAreaConstraint::GetJacobianWithRespectToLambdas() const
{
  int m = GetNumberOfConstraints();
  int n = ee_load_.GetOptimizationVariables().size();
  Jacobian jac_(m, n);

  int row_idx = 0;
  int col_idx = 0;
  for (double t : dts_) {
    for (auto f : ee_motion_.GetContacts(t)) {
      for (auto dim : d2::AllDimensions)
        jac_.insert(row_idx+dim,col_idx) = f.p(dim);

      col_idx++;
    }

    row_idx += kDim2d;
  }

  return jac_;
}

SupportAreaConstraint::Jacobian
SupportAreaConstraint::GetJacobianWithRespectToContacts () const
{
  int row_idx = 0;
  int m = GetNumberOfConstraints();
//  int n = footholds_.size() * kDim2d;
  int n = ee_motion_.GetAllFreeContacts().size() *kDim2d;

  Jacobian jac_(m, n);

  int idx_lambdas = 0;

  for (double t : dts_) {
    for (auto c : ee_motion_.GetContacts(t)) {
      if (c.id != ContactBase::kFixedByStartStance) {
        for (auto dim : d2::AllDimensions) {
          int idx_contact = ee_motion_.Index(c.ee, c.id, dim);
          jac_.insert(row_idx+dim, idx_contact) = ee_load_.GetOptimizationVariables()(idx_lambdas);
        }
      }

      idx_lambdas++;
    }

    row_idx    += kDim2d;
  }

  return jac_;
}

SupportAreaConstraint::Jacobian
SupportAreaConstraint::GetJacobianWithRespectToCop () const
{
    int m = GetNumberOfConstraints();
    int n   = cop_.GetOptimizationVariables().rows();
    Jacobian jac_(m, n);
    jac_.setIdentity();
    jac_ = -1*jac_;

    return jac_;
}

SupportAreaConstraint::Jacobian
SupportAreaConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empty matrix

  if (var_set == CenterOfPressure::ID) {
    jac = GetJacobianWithRespectToCop();
  }

  if (var_set == EndeffectorsMotion::ID) {
    jac = GetJacobianWithRespectToContacts();
  }

  if (var_set == EndeffectorLoad::ID) {
    jac = GetJacobianWithRespectToLambdas();
  }

  return jac;
}


} /* namespace opt */
} /* namespace xpp */
