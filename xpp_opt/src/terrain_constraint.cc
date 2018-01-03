/**
 @file    terrain_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#include <xpp_opt/constraints/terrain_constraint.h>

#include <array>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <vector>

#include <xpp_opt/variables/node_values.h>
#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/state.h>


namespace xpp {

using namespace opt;

TerrainConstraint::TerrainConstraint (const HeightMap::Ptr& terrain,
                                      std::string ee_motion)
    :ConstraintSet(kSpecifyLater, "Terrain-Constraint-" + ee_motion)
{

  ee_motion_id_ = ee_motion;
//  ee_motion_ = opt_vars->GetComponent<EEMotionNodes>(ee_motion);

  terrain_ = terrain;

//  AddOptimizationVariables(opt_vars);


//  SetName("Terrain-Constraint-" + ee_motion);
}

void
TerrainConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_motion_ = x->GetComponent<EEMotionNodes>(ee_motion_id_);

  // skip first node, b/c already constrained by initial stance
  for (int id=1; id<ee_motion_->GetNodes().size(); ++id)
    node_ids_.push_back(id);

  int constraint_count = node_ids_.size();
  SetRows(constraint_count);
}

VectorXd
TerrainConstraint::GetValues () const
{
  VectorXd g(GetRows());

  auto nodes = ee_motion_->GetNodes();
  int row = 0;
  for (int id : node_ids_) {
    Vector3d p = nodes.at(id).at(kPos);
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
    if (ee_motion_->IsContactNode(id))
      bounds.at(row) = BoundZero;
    else
      bounds.at(row) = Bounds(0.0, max_z_distance_above_terrain_);
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

      Vector3d p = nodes.at(id).at(kPos);
      for (auto dim : {X,Y})
        jac.coeffRef(row, ee_motion_->Index(id, kPos, dim)) = -terrain_->GetDerivativeOfHeightWrt(To2D(dim), p.x(), p.y());

      row++;
    }
  }
}

TerrainConstraint::~TerrainConstraint ()
{
}

} /* namespace xpp */
