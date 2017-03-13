/**
 @file    polygon_center_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/polygon_center_constraint.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/opt/variable_names.h>

namespace xpp {
namespace opt {

PolygonCenterConstraint::PolygonCenterConstraint ()
{
  // TODO Auto-generated constructor stub
}

PolygonCenterConstraint::~PolygonCenterConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
PolygonCenterConstraint::Init (const MotionStructure& motion_structure)
{
  for (const auto& node : motion_structure.GetPhaseStampedVec()) {
    int contacts_fixed = node.contacts_fixed_.size();
    int contacts_free = node.contacts_opt_.size();
    n_contacts_per_node_.push_back(contacts_fixed + contacts_free);
  }
}

void
PolygonCenterConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  lambdas_ = opt_var->GetVariables(VariableNames::kConvexity);
}

PolygonCenterConstraint::VectorXd
PolygonCenterConstraint::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  int idx = 0;

  for (int m : n_contacts_per_node_) {

    double g_node = 0;
    for (int j=0; j<m; ++j) {
      double lamb_j = lambdas_(idx+j);
      g_node += std::pow(lamb_j,2) - 2./m*lamb_j;
    }

    g_vec.push_back(g_node);
    idx += m;
  }

  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

VecBound
PolygonCenterConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  for (int m : n_contacts_per_node_)
    bounds.push_back(Bound(-1./m, -1./m)); // should lie in center of polygon

  return bounds;
}

PolygonCenterConstraint::Jacobian
PolygonCenterConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  if (var_set == VariableNames::kConvexity) {
    int col_idx = 0;
    int row_idx = 0;
    jac = Jacobian(n_contacts_per_node_.size(), lambdas_.rows());

    for (int m : n_contacts_per_node_) {
      for (int j=0; j<m; j++) {
        double idx = col_idx+j;
        jac.insert(row_idx,idx) = 2*(lambdas_(idx)-1./m);
      }

      col_idx += m;
      row_idx++;
    }
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
