/**
 @file    terrain_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#include <xpp/constraints/terrain_constraint.h>

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


TerrainConstraint::TerrainConstraint (const HeightMap::Ptr& terrain,
                                      const OptVarsPtr& opt_vars,
                                      std::string ee_motion)
{
  ee_motion_ = opt_vars->GetComponent<EndeffectorNodes>(ee_motion);
  terrain_ = terrain;

  AddOptimizationVariables(opt_vars);

  int constraint_count = ee_motion_->GetNodes().size(); // z position of every node
  SetRows(constraint_count);
  SetName("Terrain-Constraint-" + ee_motion);
}

VectorXd
TerrainConstraint::GetValues () const
{
  VectorXd g(GetRows());

  auto nodes = ee_motion_->GetNodes();
  for (int i=0; i<nodes.size(); ++i) {
    Vector3d p = nodes.at(i).at(kPos);
    g(i) = p.z() - terrain_->GetHeight(p.x(), p.y());
  }

  return g;
}

VecBound
TerrainConstraint::GetBounds () const
{
  VecBound bounds(GetRows());

  for (int i=0; i<ee_motion_->GetNodes().size(); ++i) {
    if (ee_motion_->IsContactNode(i))
      bounds.at(i) = kEqualityBound_;
    else
      bounds.at(i) = Bound(0.0, max_z_distance_above_terrain_);
  }

  return bounds;
}

void
TerrainConstraint::FillJacobianWithRespectTo (std::string var_set,
                                              Jacobian& jac) const
{
  if (var_set == ee_motion_->GetName()) {

    auto nodes = ee_motion_->GetNodes();
    for (int i=0; i<nodes.size(); ++i) {

      jac.coeffRef(i, ee_motion_->Index(i, kPos, Z)) = 1.0;

      Vector3d p = nodes.at(i).at(kPos);
      jac.coeffRef(i, ee_motion_->Index(i, kPos, Y)) = -terrain_->GetHeightDerivWrtY(p.x(), p.y());
      jac.coeffRef(i, ee_motion_->Index(i, kPos, X)) = -terrain_->GetHeightDerivWrtX(p.x(), p.y());
    }
  }
}

TerrainConstraint::~TerrainConstraint ()
{
}

} /* namespace opt */
} /* namespace xpp */
