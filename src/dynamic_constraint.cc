/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/dynamic_constraint.h>
#include <xpp/opt/optimization_variables.h>
#include <xpp/opt/com_motion.h>

namespace xpp {
namespace opt {

using namespace xpp::utils;

DynamicConstraint::DynamicConstraint ()
{
  // TODO Auto-generated constructor stub

}

DynamicConstraint::~DynamicConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
DynamicConstraint::Init (const ComMotion& com_motion,
                         const MotionStructure& motion_structure)
{
  com_motion_ = com_motion.clone();
  motion_structure_ = motion_structure;
}

void
DynamicConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(VariableNames::kSplineCoeff);
  com_motion_->SetCoefficients(x_coeff);
  cop_     = opt_var->GetVariables(VariableNames::kCenterOfPressure);
}

DynamicConstraint::VectorXd
DynamicConstraint::EvaluateConstraint () const
{
  int m = cop_.size();
  Eigen::VectorXd g(m);

  static const double height = 0.58; // zmp_ make parameter
  int n = 0;
  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {

    auto com = com_motion_->GetCom(node.time_);
//    com.v.setZero(); // zmp_ now it should be the same as zmp constraint
    model_.SetCurrent(com.p, com.v, height);

    // acceleration as predefined by physics
    Eigen::Vector2d acc_physics = model_.GetDerivative(cop_.middleRows<kDim2d>(kDim2d*n));

    g.middleRows<kDim2d>(kDim2d*n) = acc_physics - com.a;
    n++;
  }

  return g;
}

DynamicConstraint::VecBound
DynamicConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  int m = EvaluateConstraint().rows();
  for (int i=0; i<m; ++i)
    bounds.push_back(kEqualityBound_);


  return bounds;
}

DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empty matrix

  if (var_set == VariableNames::kCenterOfPressure) {
//    jac = GetJacobianWithRespectToCop();
  }

  if (var_set == VariableNames::kSplineCoeff) {
//    jac = GetJacobianWithRespectToContacts();
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
