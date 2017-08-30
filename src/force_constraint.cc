/**
 @file    force_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#include <xpp/constraints/force_constraint.h>

#include <array>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>
#include <xpp/variables/node_values.h>

namespace xpp {
namespace opt {


ForceConstraint::ForceConstraint (const HeightMap::Ptr& terrain,
                                  double friction_coeff,
                                  const OptVarsPtr& opt_vars,
                                  const std::string& ee_force_id,
                                  const std::string& ee_motion_id)
{
  ee_force_  = opt_vars->GetComponent<EndeffectorNodes>(ee_force_id);
  ee_motion_ = opt_vars->GetComponent<EndeffectorNodes>(ee_motion_id);
  terrain_ = terrain;
  friction_coeff_ = friction_coeff;

  AddOptimizationVariables(opt_vars);

  int constraint_count = ee_force_->GetNodes().size(); // z position of every node
  SetRows(constraint_count);
  SetName("Force-Constraint-" + ee_force_id);
}

VectorXd
ForceConstraint::GetValues () const
{
  VectorXd g(GetRows());

  auto nodes = ee_force_->GetNodes();
  for (int i=0; i<nodes.size(); ++i) {
    Vector3d f = nodes.at(i).at(kPos);
    g(i) = f.z() - terrain_->GetHeight(f.x(), f.y());
  }

  return g;
}

VecBound
ForceConstraint::GetBounds () const
{
  VecBound bounds(GetRows());

  for (int i=0; i<ee_force_->GetNodes().size(); ++i) {
    if (ee_force_->IsContactNode(i))
      bounds.at(i) = kEqualityBound_;
    else
      bounds.at(i) = Bound(0.0, 1.0);
  }

  return bounds;
}

void
ForceConstraint::FillJacobianWithRespectTo (std::string var_set,
                                              Jacobian& jac) const
{
  if (var_set == ee_force_->GetName()) {

    auto nodes = ee_force_->GetNodes();
    for (int i=0; i<nodes.size(); ++i) {

      jac.coeffRef(i, ee_force_->Index(i, kPos, Z)) = 1.0;

      Vector3d p = nodes.at(i).at(kPos);
      jac.coeffRef(i, ee_force_->Index(i, kPos, Y)) = -terrain_->GetHeightDerivWrtY(p.x(), p.y());
      jac.coeffRef(i, ee_force_->Index(i, kPos, X)) = -terrain_->GetHeightDerivWrtX(p.x(), p.y());
    }
  }
}

ForceConstraint::~ForceConstraint ()
{
}

} /* namespace opt */
} /* namespace xpp */
