/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/dynamic_constraint.h>

#include <Eigen/Dense>
#include <vector>

#include <xpp/endeffectors.h>
#include <xpp/state.h>

#include <xpp/opt/variables/endeffectors_force.h>
#include <xpp/opt/variables/endeffectors_motion.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint (const OptVarsPtr& opt_vars,
                                      double T,
                                      double dt)
    :TimeDiscretizationConstraint(T, dt, opt_vars)
{
  model_ = std::make_shared<CentroidalModel>();
//  model_ = std::make_shared<LIPModel>();

  SetName("DynamicConstraint");
  base_linear_  = std::dynamic_pointer_cast<PolynomialSpline>  (opt_vars->GetComponent(id::base_linear));
  base_angular_ = std::dynamic_pointer_cast<PolynomialSpline>  (opt_vars->GetComponent(id::base_angular));
//  ee_motion_    = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetComponent(id::endeffectors_motion));
  ee_load_      = std::dynamic_pointer_cast<EndeffectorsForce> (opt_vars->GetComponent(id::endeffector_force));

  auto ee_ordered = ee_load_->GetForce(0.0).GetEEsOrdered();
  for (auto ee :  ee_ordered) {
    std::string id = id::endeffectors_motion+std::to_string(ee);
    ee_splines_.push_back(std::dynamic_pointer_cast<EndeffectorSpline>(opt_vars->GetComponent(id)));
  }


  SetRows(GetNumberOfNodes()*kDim6d);

  converter_ = AngularStateConverter(base_angular_);
}

int
DynamicConstraint::GetRow (int node, Coords6D dimension) const
{
  return kDim6d*node + dimension;
}

DynamicConstraint::~DynamicConstraint ()
{
}

void
DynamicConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{
  // acceleration the system should have given by physics
  UpdateModel(t);
  Vector6d acc_model = model_->GetBaseAcceleration();

  // acceleration base has with current values of optimization variables
  // angular acceleration fixed to zero for now
  Vector6d acc_parametrization = Vector6d::Zero();
  acc_parametrization.middleRows(AX, kDim3d) = converter_.GetAngularAcceleration(t);
  acc_parametrization.middleRows(LX, kDim3d) = base_linear_->GetPoint(t).a_;

  for (auto dim : AllDim6D)
    g(GetRow(k,dim)) = acc_model(dim) - acc_parametrization(dim);
}

void
DynamicConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{
  for (auto dim : AllDim6D) {
    if (dim == LZ)
      bounds.at(GetRow(k,dim)) = Bound(kGravity, kGravity);
    else
      bounds.at(GetRow(k,dim)) = kEqualityBound_;
  }
}

void
DynamicConstraint::UpdateJacobianAtInstance(double t, int k, Jacobian& jac,
                                            std::string var_set) const
{
  UpdateModel(t);

  int row = GetRow(k,AX);

  for (auto ee : model_->GetEEIDs()) {
    if (var_set == ee_load_->GetName())
      jac.middleRows(row, kDim6d) += model_->GetJacobianofAccWrtForce(*ee_load_, t, ee);

    if (var_set == ee_splines_.at(ee)->GetName()) {
      Jacobian jac_ee_pos = ee_splines_.at(ee)->GetJacobian(t,kPos);
      jac.middleRows(row, kDim6d) = model_->GetJacobianofAccWrtEEPos(jac_ee_pos, ee);
    }
  }


  if (var_set == base_linear_->GetName()) {
    Jacobian jac_model = model_->GetJacobianOfAccWrtBaseLin(*base_linear_, t);

    Jacobian jac_parametrization(kDim6d, base_linear_->GetRows());
    jac_parametrization.middleRows(LX, kDim3d) = base_linear_->GetJacobian(t,kAcc);

    jac.middleRows(row, kDim6d) = jac_model - jac_parametrization;
  }


  // zmp_ DRY with above
  if (var_set == base_angular_->GetName()) {

    Jacobian jac_model = model_->GetJacobianOfAccWrtBaseAng(*base_angular_, t);

    Jacobian jac_parametrization(kDim6d, base_angular_->GetRows());
    jac_parametrization.middleRows(AX, kDim3d) = converter_.GetDerivOfAngAccWrtCoeff(t);

    jac.middleRows(row, kDim6d) = jac_model - jac_parametrization;
  }
}

void
DynamicConstraint::UpdateModel (double t) const
{
  auto com_pos = base_linear_->GetPoint(t).p_;
  auto ee_load = ee_load_->GetForce(t);

  EndeffectorsPos ee_pos(ee_load.GetCount());
  for (auto ee :  ee_pos.GetEEsOrdered()) {
    ee_pos.At(ee) = ee_splines_.at(ee)->GetPoint(t).p_;
  }

//  auto ee_pos  = ee_motion_->GetEndeffectors(t).GetPos();
  model_->SetCurrent(com_pos, ee_load, ee_pos);
}

} /* namespace opt */
} /* namespace xpp */

