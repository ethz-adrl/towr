/**
 @file    terrain_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#include <towr/constraints/terrain_constraint.h>

#include <array>
#include <memory>
#include <vector>
#include <Eigen/Eigen>

#include <towr/variables/node_variables.h>


namespace towr {


TerrainConstraint::TerrainConstraint (const HeightMap::Ptr& terrain,
                                      std::string ee_motion)
    :ConstraintSet(kSpecifyLater, "Terrain-Constraint-" + ee_motion)
{
  ee_motion_id_ = ee_motion;
  terrain_ = terrain;
}

void
TerrainConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_motion_ = x->GetComponent<PhaseNodes>(ee_motion_id_);

  // skip first node, b/c already constrained by initial stance
  for (int id=1; id<ee_motion_->GetNodes().size(); ++id)
    node_ids_.push_back(id);

  int constraint_count = node_ids_.size();
  SetRows(constraint_count);
}

Eigen::VectorXd
TerrainConstraint::GetValues () const
{
  VectorXd g(GetRows());

  auto nodes = ee_motion_->GetNodes();
  int row = 0;
  for (int id : node_ids_) {
    Vector3d p = nodes.at(id).p();
    g(row++) = p.z() - terrain_->GetHeight(p.x(), p.y());
  }

  return g;
}

TerrainConstraint::VecBound
TerrainConstraint::GetBounds () const
{
  VecBound bounds(GetRows());

  int row = 0;
  for (int id : node_ids_) {
    if (ee_motion_->IsConstantNode(id))
      bounds.at(row) = ifopt::BoundZero;
    else
      bounds.at(row) = ifopt::Bounds(0.0, max_z_distance_above_terrain_);
    row++;
  }

  return bounds;
}

void
TerrainConstraint::FillJacobianBlock (std::string var_set,
                                      Jacobian& jac) const
{
  if (var_set == ee_motion_->GetName()) {

    auto nodes = ee_motion_->GetNodes();
    int row = 0;
    for (int id : node_ids_) {

      jac.coeffRef(row, ee_motion_->Index(id, kPos, Z)) = 1.0;

      Vector3d p = nodes.at(id).p();
      for (auto dim : {X,Y})
        jac.coeffRef(row, ee_motion_->Index(id, kPos, dim)) = -terrain_->GetDerivativeOfHeightWrt(To2D(dim), p.x(), p.y());

      row++;
    }
  }
}

} /* namespace towr */
