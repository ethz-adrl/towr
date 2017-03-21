/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/dynamic_constraint.h>
#include <xpp/opt/com_motion.h>
#include <xpp/opt/variable_names.h>

namespace xpp {
namespace opt {

using Vector2d = Eigen::Vector2d;
using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;

DynamicConstraint::DynamicConstraint ()
{
  kHeight_ = 0.0;
  name_ = "Dynamic";
}

DynamicConstraint::~DynamicConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
DynamicConstraint::Init (const ComMotion& com_motion,
                         const CenterOfPressure& cop, double dt)
{
  com_motion_ = com_motion.clone();
  kHeight_ = com_motion.GetZHeight();
  cop_ = cop;

  double t = 0.0;
  dts_.clear();
  for (int i=0; i<floor(com_motion.GetTotalTime()/dt); ++i) {
    dts_.push_back(t);
    t += dt;
  }
}

void
DynamicConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(VariableNames::kSplineCoeff);
  com_motion_->SetCoefficients(x_coeff);

  VectorXd cop = opt_var->GetVariables(VariableNames::kCenterOfPressure);
  cop_.SetOptimizationVariables(cop);
}

DynamicConstraint::VectorXd
DynamicConstraint::EvaluateConstraint () const
{
  int m = cop_.GetOptimizationVariables().size();
  Eigen::VectorXd g(m);

  for (int k=0; k<dts_.size(); ++k) {

    auto com = com_motion_->GetCom(dts_.at(k));
    model_.SetCurrent(com.p, com.v, kHeight_);

    // acceleration as predefined by physics
    // zmp_ create class for CoP as well
    Vector2d acc_physics = model_.GetDerivative(cop_.GetOptimizationVariables().middleRows<kDim2d>(kDim2d*k));
    g.middleRows<kDim2d>(kDim2d*k) = acc_physics - com.a;
  }

  return g;
}

VecBound
DynamicConstraint::GetBounds () const
{
  std::vector<Bound> bounds;
  int m = EvaluateConstraint().rows();
  for (int i=0; i<m; ++i)
    bounds.push_back(kEqualityBound_);

  return bounds;
}

DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWrtCop () const
{
  int m = GetNumberOfConstraints();
  Jacobian jac(m, cop_.GetOptimizationVariables().rows());

  int row=0;
  for (double t : dts_) {

    auto com = com_motion_->GetCom(t);
    model_.SetCurrent(com.p, com.v, kHeight_);

    for (auto dim : {X, Y})
      jac.insert(row+dim,row+dim) = model_.GetJacobianApproxWrtCop(dim);

    row += kDim2d;
  }

  return jac;
}

DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWrtCom () const
{
  int m = GetNumberOfConstraints();
  Jacobian jac(m, com_motion_->GetTotalFreeCoeff());

  int n=0;
  for (double t : dts_) {

    auto com = com_motion_->GetCom(t);
    model_.SetCurrent(com.p, com.v, kHeight_);
    Vector2d cop = cop_.GetOptimizationVariables().middleRows<kDim2d>(kDim2d*n);

    for (auto dim : {X, Y}) {
      Jacobian jac_acc     = com_motion_->GetJacobian(t,kAcc,dim);
      Jacobian jac_physics = model_.GetJacobianApproxWrtSplineCoeff(*com_motion_, t, dim, cop);
      jac.row(kDim2d*n + dim) = jac_physics - jac_acc;
    }

    n++;
  }

  return jac;
}

DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empty matrix

  if (var_set == VariableNames::kCenterOfPressure)
    jac = GetJacobianWrtCop();

  if (var_set == VariableNames::kSplineCoeff)
    jac = GetJacobianWrtCom();

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
