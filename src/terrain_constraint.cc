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

TerrainConstraint::TerrainConstraint (const OptVarsPtr& opt_vars,
                                      std::string ee_nodes_id)
{
  node_values_ = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(ee_nodes_id));


  AddOptimizationVariables(opt_vars);
  SetRows(0);
}

VectorXd
TerrainConstraint::GetValues () const
{
  VectorXd g(GetRows());


  auto nodes = node_values_->GetNodes();

  // shouldn't need to know anything about optimization indices here
  // just if step or stance node
  for (const auto& n : nodes) {

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
  // here knowledge about indices will be neccessary
  if (var_set == node_values_->GetName()) {

  }
}

TerrainConstraint::~TerrainConstraint ()
{
}

} /* namespace opt */
} /* namespace xpp */
