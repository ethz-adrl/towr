/**
 @file    swing_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#include <towr/constraints/swing_constraint.h>

#include <array>
#include <memory>
#include <vector>
#include <Eigen/Eigen>

#include <towr/variables/node_variables.h>
#include "../include/towr/variables/cartesian_dimensions.h"

namespace towr {


SwingConstraint::SwingConstraint (std::string ee_motion)
    :ConstraintSet(kSpecifyLater, "Swing-Constraint-" + ee_motion)
{
  ee_motion_id_ = ee_motion;
}

void
towr::SwingConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_motion_ = x->GetComponent<PhaseNodes>(ee_motion_id_);

  pure_swing_node_ids_ = ee_motion_->GetIndicesOfNonConstantNodes();
  pure_swing_node_ids_.erase(pure_swing_node_ids_.begin()); // skip first node
  pure_swing_node_ids_.pop_back(); // because swinging last node has no further node

  // constrain xy position and velocity of every swing node
  // add +1 per node if swing in apex is constrained
  int constraint_count =  pure_swing_node_ids_.size()*2*k2D;

  SetRows(constraint_count);
}

Eigen::VectorXd
SwingConstraint::GetValues () const
{
  VectorXd g(GetRows());

  int row = 0;
  auto nodes = ee_motion_->GetNodes();
  for (int node_id : pure_swing_node_ids_) {

    // assumes two splines per swingphase and starting and ending in stance
    auto curr = nodes.at(node_id);

    Vector2d prev = nodes.at(node_id-1).p().topRows<k2D>();
    Vector2d next = nodes.at(node_id+1).p().topRows<k2D>();

    Vector2d distance_xy    = next - prev;
    Vector2d xy_center      = prev + 0.5*distance_xy;
    Vector2d des_vel_center = distance_xy/t_swing_avg_; // linear interpolation not accurate
    for (auto dim : {X,Y}) {
      g(row++) = curr.p()(dim) - xy_center(dim);
      g(row++) = curr.v()(dim) - des_vel_center(dim);
    }
    //      g(row++) = curr.pos.z() - swing_height_in_world_;
  }

  return g;
}

SwingConstraint::VecBound
SwingConstraint::GetBounds () const
{
  return VecBound(GetRows(), ifopt::BoundZero);
}

void
SwingConstraint::FillJacobianBlock (std::string var_set,
                                    Jacobian& jac) const
{
  if (var_set == ee_motion_->GetName()) {

    int row = 0;
    for (int node_id : pure_swing_node_ids_) {

      for (auto dim : {X,Y}) {
        // position constraint
        jac.coeffRef(row, ee_motion_->Index(node_id,   kPos, dim)) =  1.0;  // current node
        jac.coeffRef(row, ee_motion_->Index(node_id+1, kPos, dim)) = -0.5;  // next node
        jac.coeffRef(row, ee_motion_->Index(node_id-1, kPos, dim)) = -0.5;  // previous node
        row++;

        // velocity constraint
        jac.coeffRef(row, ee_motion_->Index(node_id,   kVel, dim)) =  1.0;              // current node
        jac.coeffRef(row, ee_motion_->Index(node_id+1, kPos, dim)) = -1.0/t_swing_avg_; // next node
        jac.coeffRef(row, ee_motion_->Index(node_id-1, kPos, dim)) = +1.0/t_swing_avg_; // previous node
        row++;
      }
      //        jac.coeffRef(row, ee_motion_->Index(i, kPos, Z)) =  1.0;  // current node
      //        row++;
    }
  }
}

} /* namespace towr */
