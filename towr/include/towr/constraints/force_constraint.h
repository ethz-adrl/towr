/**
 @file    force_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#ifndef TOWR_CONSTRAINTS_FORCE_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_FORCE_CONSTRAINT_H_

#include <string>

#include <ifopt/constraint_set.h>

#include <towr/variables/phase_nodes.h>
#include <towr/height_map.h>

namespace towr {

/** Ensures the end-effector force lies inside friction cone.
 *
 * Attention: This is enforced only at the spline nodes.
 */
class ForceConstraint : public ifopt::ConstraintSet {
public:
  using EndeffectorID = uint;
  using Vector3d = Eigen::Vector3d;

  ForceConstraint (const HeightMap::Ptr& terrain,
                   double force_limit_in_normal_direction,
                   EndeffectorID ee);
  virtual ~ForceConstraint () = default;

  virtual void InitVariableDependedQuantities(const VariablesPtr& x) override;

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;


private:
  PhaseNodes::Ptr ee_force_;
  PhaseNodes::Ptr ee_motion_;

  HeightMap::Ptr terrain_;
  double force_limit_normal_direction_;
  double mu_; // friction coeff
  int n_constraints_per_node_;

  EndeffectorID ee_;
  std::vector<int> pure_stance_force_node_ids_;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_FORCE_CONSTRAINT_H_ */
