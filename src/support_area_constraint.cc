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
SupportAreaConstraint::Init (const EndeffectorsMotion& ee_motion,
                             const EndeffectorLoad& ee_load,
                             const CenterOfPressure& cop,
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
}

void
SupportAreaConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  // zmp_ automate with base class...
  VectorXd lambdas   = opt_var->GetVariables(EndeffectorLoad::ID);
  VectorXd footholds = opt_var->GetVariables(EndeffectorsMotion::ID);
  VectorXd cop       = opt_var->GetVariables(CenterOfPressure::ID);

  ee_motion_.SetOptimizationParameters(footholds);
  ee_load_.SetOptimizationVariables(lambdas);
  cop_.SetOptimizationVariables(cop);
}

SupportAreaConstraint::VectorXd
SupportAreaConstraint::EvaluateConstraint () const
{
  int m = dts_.size() * kDim2d; // DRY with dynamic constraint
  Eigen::VectorXd g(m);

  int k = 0;
  for (double t : dts_) {

    Vector2d convex_contacts = Vector2d::Zero();

//    int c = 0; // DRY number of contact

//    std::cout << "t: " << t << std::endl;

    // spring_clean_ DRY with these two functions, super ugly... :-(
    auto lambda_k = ee_load_.GetLoadValues(t);

//    std::cout << "lambda_k.size(): " << lambda_k.size() << std::endl;
//    std::cout << "ee_motion_.size(): " <<  ee_motion_.GetContacts(t).size() << std::endl;


    // spring_clean_ could actually also be all the endeffectors, then contact flags would only
    // be in other constraint
    for (auto f : ee_motion_.GetContacts(t))
      convex_contacts += lambda_k.At(f.ee)*f.p.topRows<kDim2d>();

    Vector2d cop = cop_.GetCop(t);
    g.middleRows<kDim2d>(kDim2d*k) = convex_contacts - cop;
    k++;
  }

  return g;
}

VecBound
SupportAreaConstraint::GetBounds () const
{
  return VecBound(dts_.size()*kDim2d, kEqualityBound_);
}

SupportAreaConstraint::Jacobian
SupportAreaConstraint::GetJacobianWithRespectToLambdas() const
{
  int m = GetNumberOfConstraints();
  int n = ee_load_.GetOptVarCount();
  Jacobian jac_(m, n);

  int row_idx = 0;
  for (double t : dts_) {

//    int c = 0; // DRY number of contact
    for (auto f : ee_motion_.GetContacts(t)) {

      for (auto dim : d2::AllDimensions) {
        int idx = ee_load_.Index(t,f.ee);
        jac_.insert(row_idx+dim,idx) = f.p(dim);
      }
//      c++;
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
  int n = ee_motion_.GetOptVarCount();

  Jacobian jac_(m, n);

  for (double t : dts_) {

//    int c = 0; // DRY number of contact
    auto lambda_k = ee_load_.GetLoadValues(t);
    for (auto f : ee_motion_.GetContacts(t)) {
      if (f.id != ContactBase::kFixedByStartStance) {
        for (auto dim : d2::AllDimensions) {
          int idx_contact = ee_motion_.Index(f.ee, f.id, dim);
          jac_.insert(row_idx+dim, idx_contact) = lambda_k.At(f.ee);
        }
      }
//      c++;
    }

    row_idx += kDim2d;
  }

  return jac_;
}

SupportAreaConstraint::Jacobian
SupportAreaConstraint::GetJacobianWithRespectToCop () const
{
  int m = GetNumberOfConstraints();
  int n   = cop_.GetOptVarCount();
  Jacobian jac(m, n);

  int row = 0;
  for (double t : dts_)
    for (auto dim : d2::AllDimensions)
      jac.row(row++) = -1 * cop_.GetJacobianWrtCop(t,dim);

  return jac;
}

SupportAreaConstraint::Jacobian
SupportAreaConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empty matrix

  // zmp_ automate this with base class as well
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
