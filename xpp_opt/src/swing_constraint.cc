/**
 @file    swing_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#include <../include/xpp_opt/constraints/swing_constraint.h>

#include <array>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <vector>

#include <../include/xpp_opt/variables/node_values.h>
#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/state.h>


namespace xpp {


SwingConstraint::SwingConstraint (const OptVarsPtr& opt_vars, std::string ee_motion)
    :Constraint(opt_vars)
{
  ee_motion_ = opt_vars->GetComponent<EEMotionNodes>(ee_motion);

//  AddOptimizationVariables(opt_vars);

  auto nodes = ee_motion_->GetNodes();
  usable_nodes_ = nodes.size() - 1; // because swinging last node has no further node


  int constraint_count = 0;
  for (int i=node_start_; i<usable_nodes_; ++i)
    if (!ee_motion_->IsContactNode(i)) {
      constraint_count += 2*kDim2d; // constrain xy position and velocity of every swing node
//      constraint_count += 1; // if swing in apex is constrained
    }

  SetName("Swing-Constraint-" + ee_motion);
  SetRows(constraint_count);
}

VectorXd
SwingConstraint::GetValues () const
{
  VectorXd g(GetRows());


  int row = 0;
  auto nodes = ee_motion_->GetNodes();
  for (int i=node_start_; i<usable_nodes_; ++i) {
    if (!ee_motion_->IsContactNode(i)) {

      // assumes two splines per swingphase and starting and ending in stance
      auto curr = nodes.at(i);

      Vector2d prev = nodes.at(i-1).at(kPos).topRows<kDim2d>();;
      Vector2d next = nodes.at(i+1).at(kPos).topRows<kDim2d>();

      Vector2d distance_xy    = next - prev;
      Vector2d xy_center      = prev + 0.5*distance_xy;
      Vector2d des_vel_center = distance_xy/t_swing_avg_; // linear interpolation not accurate
      for (auto dim : {X,Y}) {
        g(row++) = curr.at(kPos)(dim) - xy_center(dim);
        g(row++) = curr.at(kVel)(dim) - des_vel_center(dim);
      }

//      g(row++) = curr.at(kPos).z() - swing_height_in_world_;

    }
  }

  return g;
}

VecBound
SwingConstraint::GetBounds () const
{
  return VecBound(GetRows(), BoundZero);
}

void
SwingConstraint::FillJacobianWithRespectTo (std::string var_set,
                                            Jacobian& jac) const
{
  if (var_set == ee_motion_->GetName()) {

    int row = 0;
    for (int i=node_start_; i<usable_nodes_; ++i) {
      if (!ee_motion_->IsContactNode(i)) { // swing-phase

        for (auto dim : {X,Y}) {
          // position constraint
          jac.coeffRef(row, ee_motion_->Index(i,   kPos, dim)) =  1.0;  // current node
          jac.coeffRef(row, ee_motion_->Index(i+1, kPos, dim)) = -0.5;  // next node
          jac.coeffRef(row, ee_motion_->Index(i-1, kPos, dim)) = -0.5;  // previous node
          row++;

          // velocity constraint
          jac.coeffRef(row, ee_motion_->Index(i,   kVel, dim)) =  1.0;              // current node
          jac.coeffRef(row, ee_motion_->Index(i+1, kPos, dim)) = -1.0/t_swing_avg_; // next node
          jac.coeffRef(row, ee_motion_->Index(i-1, kPos, dim)) = +1.0/t_swing_avg_; // previous node
          row++;
        }

//        jac.coeffRef(row, ee_motion_->Index(i, kPos, Z)) =  1.0;  // current node
//        row++;
      }
    }
  }
}

SwingConstraint::~SwingConstraint ()
{
}

} /* namespace xpp */
