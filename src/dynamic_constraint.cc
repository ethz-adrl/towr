/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/dynamic_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

#include <xpp/bound.h>
#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/variables/base_motion.h>
#include <xpp/opt/variables/endeffector_load.h>
#include <xpp/opt/variables/endeffectors_motion.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint (const OptVarsPtr& opt_vars,
                                      double T,
                                      double dt)
    :TimeDiscretizationConstraint(T, dt, opt_vars),
     model_(0.58) // constant walking height for model
{
  SetName("DynamicConstraint");
  com_motion_ = std::dynamic_pointer_cast<BaseMotion>        (opt_vars->GetComponent("base_motion"));
  ee_motion_  = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetComponent("endeffectors_motion"));
  ee_load_    = std::dynamic_pointer_cast<EndeffectorLoad>   (opt_vars->GetComponent("endeffector_load"));

  ee_ids_ = ee_load_->GetLoadValues(0.0).GetEEsOrdered();
  dim_    = {d2::X, d2::Y};//  com_motion_->GetComSpline().dim_;
  SetRows(GetNumberOfNodes()*dim_.size());
}

DynamicConstraint::~DynamicConstraint ()
{
}

void
DynamicConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{
  // acceleration the system should have given by physics
  UpdateModel(t);
  Vector2d acc_model = model_.GetAcceleration();

  // current acceleration given by the optimization variables
  Vector3d acc_opt = com_motion_->GetCom(t).a_;
  for (auto dim : dim_)
    g(GetRow(k,dim)) = acc_model(dim) - acc_opt(dim);
}

void
DynamicConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{
  for (auto dim : dim_)
    bounds.at(GetRow(k,dim)) = kEqualityBound_;
}

void
DynamicConstraint::UpdateJacobianAtInstance(double t, int k, Jacobian& jac,
                                            std::string var_set) const
{
  UpdateModel(t);

  for (auto dim : dim_) {
    int row = GetRow(k,dim);

    for (auto ee : ee_ids_) {

      if (var_set == ee_load_->GetName()) {
        double deriv_load = model_.GetDerivativeOfAccWrtLoad(ee, dim);
        jac.coeffRef(row, ee_load_->Index(t,ee)) = deriv_load;
      }

      if (var_set == ee_motion_->GetName()) {
        double deriv_ee = model_.GetDerivativeOfAccWrtEEPos(ee);
        jac.row(row) += deriv_ee* ee_motion_->GetJacobianPos(t, ee, dim);
      }
    }


    if (var_set == com_motion_->GetName()) {
      Coords3D dim3d = static_cast<Coords3D>(dim);
      Jacobian jac_model = model_.GetJacobianOfAccWrtBase(*com_motion_, t, dim3d);
      Jacobian jac_opt   = com_motion_->GetJacobian(t, kAcc, dim3d);
      jac.row(row) = jac_model - jac_opt;
    }
  }
}

void
DynamicConstraint::UpdateModel (double t) const
{
  auto com     = com_motion_->GetCom(t);
  auto ee_load = ee_load_   ->GetLoadValues(t);
  auto ee_pos  = ee_motion_ ->GetEndeffectors(t).GetPos();
  model_.SetCurrent(com.p_.topRows<kDim2d>(), ee_load, ee_pos);
}

int
DynamicConstraint::GetRow (int node, int dimension) const
{
  return dim_.size()*node + dimension;
}

} /* namespace opt */
} /* namespace xpp */

