/**
 @file    support_area_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Defines the DynamicConstraint class
 */

#include <xpp/opt/constraints/support_area_constraint.h>

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

#include <xpp/bound.h>
#include <xpp/opt/variables/center_of_pressure.h>
#include <xpp/opt/variables/endeffector_load.h>
#include <xpp/opt/variables/endeffectors_motion.h>

namespace xpp {
namespace opt {

SupportAreaConstraint::SupportAreaConstraint (const OptVarsPtr& opt_vars,
                                              double dt, double T)
    :TimeDiscretizationConstraint(T, dt)
{
  name_ = "Support Area";
  ee_motion_ = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetSet("endeffectors_motion"));
  ee_load_   = std::dynamic_pointer_cast<EndeffectorLoad>   (opt_vars->GetSet("endeffector_load"));
  cop_       = std::dynamic_pointer_cast<CenterOfPressure>  (opt_vars->GetSet("center_of_pressure"));

  int num_constraints = GetNumberOfNodes()*kDim2d;
  SetDimensions(opt_vars->GetOptVarsVec(), num_constraints);
}

SupportAreaConstraint::~SupportAreaConstraint ()
{
}

int
SupportAreaConstraint::GetConstraintNr (int node, int dimension) const
{
  return node*kDim2d + dimension;
}

void
SupportAreaConstraint::UpdateConstraintAtInstance (double t, int k)
{
  auto lambda_k    = ee_load_->GetLoadValues(t);
  auto ee_state    = ee_motion_->GetEndeffectors(t);

  Vector2d cop  = cop_->GetCop(t);

  for (auto dim : d2::AllDimensions) {
    double conv = 0.0;

    for (auto ee : ee_state.GetEEsOrdered())
      conv += lambda_k.At(ee)*ee_state.At(ee).p(dim);

    g_(GetConstraintNr(k,dim)) = conv - cop(dim);
  }
}

void
SupportAreaConstraint::UpdateBoundsAtInstance (double t, int k)
{
  for (auto dim : d2::AllDimensions)
    bounds_.at(GetConstraintNr(k,dim)) = kEqualityBound_;
}

void
SupportAreaConstraint::UpdateJacobianAtInstance (double t, int k)
{
  UpdateJacobianWithRespectToLoad(t,k);
  UpdateJacobianWithRespectToEEMotion(t,k);
  UpdateJacobianWithRespectToCop(t,k); // zmp_ actually constant, so doesn't have to be here
}

void
SupportAreaConstraint::UpdateJacobianWithRespectToLoad(double t, int k)
{
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_load_->GetId());

  auto ee_state = ee_motion_->GetEndeffectors(t);

  for (auto dim : d2::AllDimensions) {
    for (auto ee : ee_state.GetEEsOrdered()) {
      int idx = ee_load_->Index(t,ee);
      jac.coeffRef(GetConstraintNr(k,dim), idx) = ee_state.At(ee).p(dim);
    }
  }
}

void
SupportAreaConstraint::UpdateJacobianWithRespectToEEMotion (double t, int k)
{
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_motion_->GetId());

  auto lambda_k = ee_load_->GetLoadValues(t);

  for (auto dim : d2::AllDimensions) {
    for (const auto ee : lambda_k.GetEEsOrdered()) {
      JacobianRow jac_ee = ee_motion_->GetJacobianWrtOptParams(t,ee,dim);

      if (ee == E0) // overwrite jacobian
        jac.row(GetConstraintNr(k,dim)) = lambda_k.At(ee) * jac_ee;
      else          // append
        jac.row(GetConstraintNr(k,dim)) += lambda_k.At(ee)* jac_ee;
    }
  }
}

void
SupportAreaConstraint::UpdateJacobianWithRespectToCop (double t, int k)
{
  Jacobian& jac = GetJacobianRefWithRespectTo(cop_->GetId());

  for (auto dim : d2::AllDimensions)
    jac.row(GetConstraintNr(k,dim)) = -1 * cop_->GetJacobianWrtCop(t,dim);
}

} /* namespace opt */
} /* namespace xpp */
