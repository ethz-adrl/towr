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

  int constraint_count = 0;
  for (int i=0; i<ee_motion_->GetNodes().size(); ++i)
    if (ee_motion_->IsContactNode(i))
      constraint_count++; // every contact node constraint in z position

  SetRows(constraint_count);
}

VectorXd
TerrainConstraint::GetValues () const
{
  VectorXd g(GetRows());

  int row = 0;
  auto nodes = ee_motion_->GetNodes();
  for (int i=0; i<nodes.size(); ++i) {
    if (ee_motion_->IsContactNode(i)) {
      Vector3d p = nodes.at(i).at(kPos);
      g(row++) = p.z() - terrain_->GetHeight(p.x(), p.y());
    }
  }

  return g;
}

VecBound
TerrainConstraint::GetBounds () const
{
  return VecBound(GetRows(), kEqualityBound_); // match height exactly
}

void
TerrainConstraint::FillJacobianWithRespectTo (std::string var_set,
                                              Jacobian& jac) const
{
  if (var_set == ee_motion_->GetName()) {

    int row = 0;
    auto nodes = ee_motion_->GetNodes();
    for (int i=0; i<nodes.size(); ++i) {
      if (ee_motion_->IsContactNode(i)) {

        jac.coeffRef(row, ee_motion_->Index(i, kPos, Z)) = 1.0;

        Vector3d p = nodes.at(i).at(kPos);
        jac.coeffRef(row, ee_motion_->Index(i, kPos, Y)) = -terrain_->GetHeightDerivWrtY(p.x(), p.y());
        jac.coeffRef(row, ee_motion_->Index(i, kPos, X)) = -terrain_->GetHeightDerivWrtX(p.x(), p.y());

        row++;
      }
    }
  }
}

TerrainConstraint::~TerrainConstraint ()
{
}

} /* namespace opt */
} /* namespace xpp */
