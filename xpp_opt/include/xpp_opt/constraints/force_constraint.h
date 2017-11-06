/**
 @file    force_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_CONSTRAINTS_FORCE_CONSTRAINT_H_
#define XPP_OPT_INCLUDE_XPP_CONSTRAINTS_FORCE_CONSTRAINT_H_

#include <string>

#include <opt_solve/composite.h>
#include <xpp_opt/height_map.h>
#include <xpp_opt/variables/phase_nodes.h>

namespace xpp {

/** Ensures the end-effector force lies inside friction cone.
 *
 * Attention: This is enforced only at the spline nodes.
 */
class ForceConstraint : public opt::Constraint {
public:
  ForceConstraint (const HeightMap::Ptr& terrain,
                   double force_limit_in_normal_direction,
                   const VariablesPtr& opt_vars,
                   const std::string& ee_force_id,
                   const std::string& ee_motion_id);
  virtual ~ForceConstraint ();

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;


private:
  EEForceNodes::Ptr ee_force_;
  EEMotionNodes::Ptr ee_motion_;

  HeightMap::Ptr terrain_;
  double force_limit_normal_direction_;
  double mu_; // friction coeff
  int n_constraints_per_node_;
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_CONSTRAINTS_FORCE_CONSTRAINT_H_ */
