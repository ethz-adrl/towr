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
using Vector2d = Eigen::Vector2d;
using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;

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
                         const MotionStructure& motion_structure,
                         double com_height)
{
  com_motion_ = com_motion.clone();
  motion_structure_ = motion_structure;
  kHeight_ = com_height;
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


  int n = 0;
  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {

    auto com = com_motion_->GetCom(node.time_);
    model_.SetCurrent(com.p, com.v, kHeight_);

    // acceleration as predefined by physics
    Vector2d acc_physics = model_.GetDerivative(cop_.middleRows<kDim2d>(kDim2d*n));
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
    int m = GetNumberOfConstraints();
    jac = Jacobian(m, cop_.rows());

    int row=0;
    for (const auto& node : motion_structure_.GetPhaseStampedVec()) {

      auto com = com_motion_->GetCom(node.time_);
      model_.SetCurrent(com.p, com.v, kHeight_);

      for (auto dim : {X, Y})
        jac.insert(row+dim,row+dim) = model_.GetJacobianApproxWrtCop(dim);

      row += kDim2d;
    }
  }

  if (var_set == VariableNames::kSplineCoeff) {
    int m = GetNumberOfConstraints();
    jac = Jacobian(m, com_motion_->GetTotalFreeCoeff());

    int n=0;
    for (const auto& node : motion_structure_.GetPhaseStampedVec()) {

      double t = node.time_;
      auto com = com_motion_->GetCom(node.time_);
      model_.SetCurrent(com.p, com.v, kHeight_);
      Vector2d cop = cop_.middleRows<kDim2d>(kDim2d*n);

      for (auto dim : {X, Y}) {
        Jacobian jac_acc     = com_motion_->GetJacobian(t,kAcc,dim);
        Jacobian jac_physics = model_.GetJacobianApproxWrtSplineCoeff(*com_motion_, t, dim, cop);
        jac.row(kDim2d*n + dim) = jac_physics - jac_acc;
      }

      n++;
    }
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
