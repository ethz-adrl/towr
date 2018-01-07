/**
 @file    swing_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#ifndef TOWR_CONSTRAINTS_SWING_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_SWING_CONSTRAINT_H_

#include <string>

#include <ifopt/composite.h>

#include <towr/variables/phase_nodes.h>

namespace towr {


class SwingConstraint : public ifopt::ConstraintSet {
public:
  SwingConstraint (std::string ee_motion_id);
  virtual ~SwingConstraint () = default;

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

  virtual void InitVariableDependedQuantities(const VariablesPtr& x) override;

private:
  EEMotionNodes::Ptr ee_motion_;
  double t_swing_avg_ = 0.3;
  int node_start_ = 1; // skip first node
  int usable_nodes_; // how many nodes are iterated over
  std::string ee_motion_id_;
//  double swing_height_in_world_ = 0.10; //  hacky way to lift legs
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_SWING_CONSTRAINT_H_ */
