/**
 @file    base_motion_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 11, 2017
 @brief   Brief description
 */

#include <towr/constraints/base_motion_constraint.h>

#include <memory>

#include <towr/variables/cartesian_declarations.h>
#include <towr/variables/variable_names.h>

namespace towr {


BaseMotionConstraint::BaseMotionConstraint (const OptimizationParameters& params,
                                            const SplineHolder& spline_holder)
    :TimeDiscretizationConstraint(params.GetTotalTime(),
                                  params.dt_base_range_of_motion_,
                                  "BaseMotionConstraint")
{
  base_linear_ = spline_holder.GetBaseLinear();
  base_angular_ = spline_holder.GetBaseAngular();

  double dev_rad = 0.1;
  node_bounds_.resize(kDim6d);
  node_bounds_.at(AX) = ifopt::NoBound;//Bounds(-dev_rad, dev_rad);
  node_bounds_.at(AY) = ifopt::NoBound;//Bounds(-dev_rad, dev_rad);
  node_bounds_.at(AZ) = ifopt::NoBound;//Bounds(-dev_rad, dev_rad); // NoBound
  node_bounds_.at(LX) = ifopt::NoBound;
  node_bounds_.at(LY) = ifopt::NoBound;//Bounds(-0.05, 0.05);

  double z_init = 0.46;//base_linear_.GetPoint(0.0).p().z();
  node_bounds_.at(LZ) = Bounds(0.46, 0.55); // allow to move dev_z cm up and down

  SetRows(GetNumberOfNodes()*node_bounds_.size());
}

void
BaseMotionConstraint::UpdateConstraintAtInstance (double t, int k,
                                                  VectorXd& g) const
{
  g.middleRows(GetRow(k, LX), kDim3d) = base_linear_->GetPoint(t).p();
  g.middleRows(GetRow(k, AX), kDim3d) = base_angular_->GetPoint(t).p();
}

void
BaseMotionConstraint::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  for (int dim=0; dim<node_bounds_.size(); ++dim)
    bounds.at(GetRow(k,dim)) = node_bounds_.at(dim);
}

void
BaseMotionConstraint::UpdateJacobianAtInstance (double t, int k,
                                                Jacobian& jac,
                                                std::string var_set) const
{
  if (var_set == id::base_ang_nodes)
    jac.middleRows(GetRow(k,AX), kDim3d) = base_angular_->GetJacobianWrtNodes(t, kPos);

  if (var_set == id::base_lin_nodes)
    jac.middleRows(GetRow(k,LX), kDim3d) = base_linear_->GetJacobianWrtNodes(t, kPos);
}

int
BaseMotionConstraint::GetRow (int node, int dim) const
{
  return node*node_bounds_.size() + dim;
}

} /* namespace towr */
