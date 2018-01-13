/**
 @file    swing_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#ifndef TOWR_CONSTRAINTS_SWING_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_SWING_CONSTRAINT_H_

#include <string>

#include <ifopt/constraint_set.h>

#include <towr/variables/phase_nodes.h>

namespace towr {


class SwingConstraint : public ifopt::ConstraintSet {
public:
  using Vector2d = Eigen::Vector2d;

  SwingConstraint (std::string ee_motion_id);
  virtual ~SwingConstraint () = default;

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

  virtual void InitVariableDependedQuantities(const VariablesPtr& x) override;

private:
  PhaseNodes::Ptr ee_motion_;
  double t_swing_avg_ = 0.3;
  std::string ee_motion_id_;
//  double swing_height_in_world_ = 0.10; //  hacky way to lift legs

  std::vector<int> pure_swing_node_ids_;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_SWING_CONSTRAINT_H_ */
