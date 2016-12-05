/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Defines the DynamicConstraint class
 */

#include <xpp/opt/dynamic_constraint.h>
#include <xpp/opt/com_motion.h>
#include <xpp/opt/optimization_variables.h>
#include <xpp/opt/zero_moment_point.h>

#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace opt {

using namespace xpp::utils;
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;

DynamicConstraint::DynamicConstraint ()
{
  // TODO Auto-generated constructor stub
}

DynamicConstraint::~DynamicConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
DynamicConstraint::Init (const ComMotion& com_motion,
                         const Contacts& contacts,
                         const MotionStructure& motion_structure)
{
  com_motion_       = com_motion.clone();
  contacts_         = ContactPtrU(new Contacts(contacts));
  motion_structure_ = motion_structure;
}

void
DynamicConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(VariableNames::kSplineCoeff);
  VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);
  lambdas_ = opt_var->GetVariables(VariableNames::kConvexity);

  com_motion_->SetCoefficients(x_coeff);
  contacts_->SetFootholdsXY(utils::ConvertEigToStd(footholds));
}

DynamicConstraint::VectorXd
DynamicConstraint::EvaluateConstraint () const
{
  int m = motion_structure_.GetPhaseStampedVec().size() * kDim2d;
  Eigen::VectorXd g(m);

  int idx_lambda = 0;
  int n = 0;
  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {

    auto com = com_motion_->GetCom(node.time_);
    Vector2d zmp = ZeroMomentPoint::CalcZmp(com.Make3D(), kWalkingHeight);



//    double n_contacts = node.phase_.free_contacts_.size() + node.phase_.fixed_contacts_.size();

    Vector2d convex_contacts;
    convex_contacts.setZero();
    for (auto contact : node.phase_.free_contacts_) {
      Vector2d p = contacts_->GetFoothold(contact.id).p.topRows<kDim2d>();
      double lamdba = lambdas_(idx_lambda++);
      convex_contacts += lamdba*p;
//      convex_contacts += 1./n_contacts*p;
    }


    for (auto contact : node.phase_.fixed_contacts_) {
      Vector2d p = contact.p.topRows<kDim2d>();
      double lamdba = lambdas_(idx_lambda++);
      convex_contacts += lamdba*p; // zmp_ DRY, put these together somehow
//      convex_contacts += 1./n_contacts*p; // zmp_ DRY, put these together somehow
    }

    g.middleRows<kDim2d>(kDim2d*n++) = convex_contacts - zmp;

  }

  return g;
}

DynamicConstraint::VecBound
DynamicConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  int m = EvaluateConstraint().rows();
  for (int i=0; i<m; ++i)
    bounds.push_back(kEqualityBound_);


  return bounds;
}

// zmp_ almost exactly the same as in convexity constraint->merge!
DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWithRespectToLambdas() const
{
  int col_idx = 0;
  int row_idx = 0;
  int m = GetNumberOfConstraints();
  int n = motion_structure_.GetTotalNumberOfNodeContacts();
  Jacobian jac_(m, n);

  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {
    int contacts_free = node.phase_.free_contacts_.size();
    int contacts_fixed = node.phase_.fixed_contacts_.size();

    for (int i=0; i<contacts_free; ++i) {
      Vector3d p = contacts_->GetFoothold(node.phase_.free_contacts_.at(i).id).p;
      for (auto dim : {X, Y})
        jac_.insert(row_idx+dim,col_idx + i) = p(dim);
    }

    col_idx += contacts_free;

    for (int i=0; i<contacts_fixed; ++i) {
      Vector3d p = node.phase_.fixed_contacts_.at(i).p;
      for (auto dim : {X, Y})
        jac_.insert(row_idx+dim,col_idx + i) = p(dim);
    }

    col_idx += contacts_fixed;
    row_idx += kDim2d;
  }

  return jac_;
}

DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWithRespectToContacts () const
{
  int row_idx = 0;
  int m = GetNumberOfConstraints();
  int n = contacts_->GetTotalFreeCoeff();
  Jacobian jac_(m, n);




  int idx_lambdas = 0;
  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {


//    double n_contacts = node.phase_.free_contacts_.size() + node.phase_.fixed_contacts_.size();

    for (auto contact : node.phase_.free_contacts_) {
      for (auto dim : {X, Y}) {
        int idx_contact = Contacts::Index(contact.id, dim);
        jac_.insert(row_idx+dim, idx_contact) = lambdas_(idx_lambdas); //1./n_contacts;
      }
      idx_lambdas++;
    }

    for (auto contact : node.phase_.fixed_contacts_) {
      idx_lambdas++; // zmp_ this is bad, depends on order free->fixed with function above...
    }

    row_idx    += kDim2d;
  }

  return jac_;
}

DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWithRespectToComMotion () const
{
  int m = GetNumberOfConstraints();
  int n   = com_motion_->GetTotalFreeCoeff();
  Jacobian jac_(m, n);

  int row=0;
  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {

    jac_.row(row++) = -1*ZeroMomentPoint::GetJacobianWrtCoeff(*com_motion_, X, kWalkingHeight, node.time_);
    jac_.row(row++) = -1*ZeroMomentPoint::GetJacobianWrtCoeff(*com_motion_, Y ,kWalkingHeight, node.time_);

  }

  return jac_;
}

DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empty matrix

  if (var_set == VariableNames::kSplineCoeff) {
    jac = GetJacobianWithRespectToComMotion();
  }

  if (var_set == VariableNames::kFootholds) {
    jac = GetJacobianWithRespectToContacts();
  }

  if (var_set == VariableNames::kConvexity) {
    jac = GetJacobianWithRespectToLambdas();
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
