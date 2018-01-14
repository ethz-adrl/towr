/**
 @file    terrain_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#ifndef TOWR_CONSTRAINTS_TERRAIN_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_TERRAIN_CONSTRAINT_H_

#include <string>

#include <ifopt/constraint_set.h>

#include <towr/variables/phase_nodes.h>
#include <towr/height_map.h>

namespace towr {

/** Ensures the endeffector position always lays on or above terrain height.
 *
 * Attention: This is enforced only at the spline nodes.
 */
class TerrainConstraint : public ifopt::ConstraintSet {
public:
  using Vector3d = Eigen::Vector3d;

  TerrainConstraint (const HeightMap::Ptr& terrain, std::string ee_motion_id);
  virtual ~TerrainConstraint () = default;


  virtual void InitVariableDependedQuantities(const VariablesPtr& x) override;

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;


private:
  PhaseNodes::Ptr ee_motion_;
  HeightMap::Ptr terrain_;

  std::string ee_motion_id_;

  std::vector<int> node_ids_;

  double max_z_distance_above_terrain_ = 1e20; // [m]
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_TERRAIN_CONSTRAINT_H_ */
