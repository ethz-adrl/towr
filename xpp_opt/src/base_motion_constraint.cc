/**
 @file    base_motion_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 11, 2017
 @brief   Brief description
 */

#include <xpp_opt/constraints/base_motion_constraint.h>

#include <memory>

#include <xpp_opt/variables/variable_names.h>

namespace xpp {

using namespace opt;

BaseMotionConstraint::BaseMotionConstraint (const OptimizationParameters& params)
    :TimeDiscretizationConstraint(params.GetTotalTime(),
                                  params.dt_base_range_of_motion_,
                                  "BaseMotionConstraint")
{
//  base_linear_  = opt_vars->GetComponent<Spline>(id::base_linear);
//  base_angular_ = opt_vars->GetComponent<Spline>(id::base_angular);

  double dev_rad = 0.1;
  node_bounds_.resize(kDim6d);
  node_bounds_.at(AX) = NoBound;//Bounds(-dev_rad, dev_rad);
  node_bounds_.at(AY) = NoBound;//Bounds(-dev_rad, dev_rad);
  node_bounds_.at(AZ) = NoBound;//Bounds(-dev_rad, dev_rad); // NoBound
  node_bounds_.at(LX) = NoBound;
  node_bounds_.at(LY) = NoBound;//Bounds(-0.05, 0.05);

  double z_init = 0.46;//base_linear_->GetPoint(0.0).p_.z();
  node_bounds_.at(LZ) = Bounds(0.46, 0.55); // allow to move dev_z cm up and down

  SetRows(GetNumberOfNodes()*node_bounds_.size());
}

void
BaseMotionConstraint::InitVariableDependedQuantities(const VariablesPtr& x)
{
  base_linear_   = x->GetComponent<Spline>(id::base_linear);
  base_angular_  = x->GetComponent<Spline>(id::base_angular);
};

void
BaseMotionConstraint::UpdateConstraintAtInstance (double t, int k,
                                                  VectorXd& g) const
{
  g.middleRows(GetRow(k, AX), kDim3d) = base_angular_->GetPoint(t).p_;
  g.middleRows(GetRow(k, LX), kDim3d) = base_linear_->GetPoint(t).p_;
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
  if (base_angular_->HoldsVarsetThatIsActiveNow(var_set, t))
    jac.middleRows(GetRow(k,AX), kDim3d) = base_angular_->GetJacobian(t, kPos);

  if (base_linear_->HoldsVarsetThatIsActiveNow(var_set, t))
    jac.middleRows(GetRow(k,LX), kDim3d) = base_linear_->GetJacobian(t, kPos);
}

int
BaseMotionConstraint::GetRow (int node, int dim) const
{
  return node*node_bounds_.size() + dim;
}

BaseMotionConstraint::~BaseMotionConstraint ()
{
  // TODO Auto-generated destructor stub
}

} /* namespace xpp */
