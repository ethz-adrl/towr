/**
 @file    convexity_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/convexity_constraint.h>
#include <xpp/opt/optimization_variables.h>

namespace xpp {
namespace opt {

ConvexityConstraint::ConvexityConstraint ()
{
  // TODO Auto-generated constructor stub
}

ConvexityConstraint::~ConvexityConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
ConvexityConstraint::Init (const MotionStructure& motion_structure)
{
  motion_structure_ = motion_structure;

  // build constant jacobian w.r.t lambdas
  int col_idx = 0;
  int row_idx = 0;
  int m = GetNumberOfConstraints();
  int n = motion_structure.GetTotalNumberOfNodeContacts();
  jac_ = Jacobian(m, n);

  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {
    int contacts_fixed = node.phase_.fixed_contacts_.size();
    int contacts_free = node.phase_.free_contacts_.size();
    int n_contacts = contacts_fixed + contacts_free;

    for (int col=0; col<n_contacts; col++)
      jac_.insert(row_idx,col_idx + col) = 1.0;

    col_idx += n_contacts;
    row_idx++;
  }
}

void
ConvexityConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  lambdas_ = opt_var->GetVariables(VariableNames::kConvexity);
}

ConvexityConstraint::VectorXd
ConvexityConstraint::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  int idx = 0;
  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {
    int contacts_fixed = node.phase_.fixed_contacts_.size();
    int contacts_free = node.phase_.free_contacts_.size();
    int n_contacts = contacts_fixed + contacts_free;

    g_vec.push_back(lambdas_.middleRows(idx, n_contacts).sum()); // sum equal to 1
    idx += n_contacts;

  }

  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

ConvexityConstraint::VecBound
ConvexityConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  for (const auto& node : motion_structure_.GetPhaseStampedVec())
    bounds.push_back(Bound(1.0, 1.0)); // sum of lambda's should equal one

  return bounds;
}

ConvexityConstraint::Jacobian
ConvexityConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  if (var_set == VariableNames::kConvexity) {
    jac = jac_;
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
