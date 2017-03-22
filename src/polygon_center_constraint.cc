/**
 @file    polygon_center_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/polygon_center_constraint.h>

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
PolygonCenterConstraint::Init (const EndeffectorLoad& ee_load)
{
  ee_load_ = ee_load;
}

void
PolygonCenterConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  Eigen::VectorXd lambdas = opt_var->GetVariables(EndeffectorLoad::ID);
  ee_load_.SetOptimizationVariables(lambdas);
}

PolygonCenterConstraint::VectorXd
PolygonCenterConstraint::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  int idx = 0;

  for (int m : ee_load_.GetContactsPerNode()) {

    double g_node = 0;
    for (int j=0; j<m; ++j) {
      double lamb_j = ee_load_.GetOptimizationVariables()(idx+j);
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
  for (int m : ee_load_.GetContactsPerNode())
    bounds.push_back(Bound(-1./m, -1./m)); // should lie in center of polygon

  return bounds;
}

PolygonCenterConstraint::Jacobian
PolygonCenterConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empty matrix

  if (var_set == EndeffectorLoad::ID) {
    int col_idx = 0;
    int row_idx = 0;
    int m = ee_load_.GetContactsPerNode().size();
    int n = ee_load_.GetOptVarCount();
    jac = Jacobian(m, n);

    for (int m : ee_load_.GetContactsPerNode()) {
      for (int j=0; j<m; j++) {
        double idx = col_idx+j;
        jac.insert(row_idx,idx) = 2*(ee_load_.GetOptimizationVariables()(idx)-1./m);
      }

      col_idx += m;
      row_idx++;
    }
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
