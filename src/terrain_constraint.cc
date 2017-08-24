/**
 @file    terrain_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#include <xpp/constraints/terrain_constraint.h>

#include <memory>

namespace xpp {
namespace opt {


HeightMap::HeightMap()
{
  slope_ = 0.7;
  slope_start_ = 0.3;
}

double
HeightMap::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (x>slope_start_)
    h = slope_*(x-slope_start_);

  return h;
}

double
HeightMap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  if (x>slope_start_)
    dhdx = slope_;

  return dhdx;
}

double
HeightMap::GetHeightDerivWrtY (double x, double y) const
{
  return 0.0;
}

HeightMap::~HeightMap()
{
}



TerrainConstraint::TerrainConstraint (const OptVarsPtr& opt_vars,
                                      std::string ee_motion)
{
  ee_motion_ = opt_vars->GetComponent<EndeffectorNodes>(ee_motion);

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
      g(row++) = p.z() - terrain_.GetHeight(p.x(), p.y());
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
        jac.coeffRef(row, ee_motion_->Index(i, kPos, Y)) = -terrain_.GetHeightDerivWrtY(p.x(), p.y());
        jac.coeffRef(row, ee_motion_->Index(i, kPos, X)) = -terrain_.GetHeightDerivWrtX(p.x(), p.y());

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
