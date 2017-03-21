/**
 @file    convexity_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/convexity_constraint.h>

namespace xpp {
namespace opt {

ConvexityConstraint::ConvexityConstraint ()
{
  name_ = "Convexity";
}

ConvexityConstraint::~ConvexityConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
ConvexityConstraint::Init (const EndeffectorLoad& ee_load)
{
//  for (const auto& node : motion_structure.GetPhaseStampedVec()) {
//    int contacts_fixed = node.contacts_fixed_.size();
//    int contacts_free = node.contacts_opt_.size();
//    n_contacts_per_node_.push_back(contacts_fixed + contacts_free);
//  }

  ee_load_ = ee_load;

  // build constant jacobian w.r.t lambdas
  int col_idx = 0;
  int row_idx = 0;
  int m = ee_load_.GetContactsPerNode().size();
  int n = ee_load_.GetOptimizationVariables().rows();
  jac_ = Jacobian(m, n);


  for (int n_contacts : ee_load_.GetContactsPerNode()) {
    for (int col=0; col<n_contacts; col++)
      jac_.insert(row_idx,col_idx + col) = 1.0;

    col_idx += n_contacts;
    row_idx++;
  }
}

void
ConvexityConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd lambdas = opt_var->GetVariables(EndeffectorLoad::ID);
  ee_load_.SetOptimizationVariables(lambdas);
}

ConvexityConstraint::VectorXd
ConvexityConstraint::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  int idx = 0;

  for (int n_contacts : ee_load_.GetContactsPerNode()) {
    g_vec.push_back(ee_load_.GetOptimizationVariables().middleRows(idx, n_contacts).sum()); // sum equal to 1
    idx += n_contacts;
  }

  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

VecBound
ConvexityConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  for (int n_contacts : ee_load_.GetContactsPerNode())
    bounds.push_back(Bound(1.0, 1.0)); // sum of lambda's should equal one

  return bounds;
}

ConvexityConstraint::Jacobian
ConvexityConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  if (var_set == EndeffectorLoad::ID) {
    jac = jac_;
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
