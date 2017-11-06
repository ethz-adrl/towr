/**
 @file    swing_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_CONSTRAINTS_SWING_CONSTRAINT_H_
#define XPP_OPT_INCLUDE_XPP_CONSTRAINTS_SWING_CONSTRAINT_H_

#include <string>

#include <xpp_solve/composite.h>
#include <xpp_opt/variables/phase_nodes.h>

namespace xpp {


class SwingConstraint : public Constraint {
public:
  SwingConstraint (const OptVarsPtr& opt_vars, std::string ee_motion_id);
  virtual ~SwingConstraint ();

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;


private:
  EEMotionNodes::Ptr ee_motion_;
  double t_swing_avg_ = 0.3;
  int node_start_ = 1; // skip first node
  int usable_nodes_; // how many nodes are iterated over
//  double swing_height_in_world_ = 0.10; //  hacky way to lift legs
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_CONSTRAINTS_SWING_CONSTRAINT_H_ */
