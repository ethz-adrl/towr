/**
 @file    convexity_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/convexity_constraint.h>

namespace xpp {
namespace opt {

ConvexityConstraint::ConvexityConstraint ()
{
  name_ = "Convexity";
}

ConvexityConstraint::~ConvexityConstraint ()
{
}

void
ConvexityConstraint::Init (const LoadPtr& ee_load)
{
//  ee_load_ = variables_.front();//ee_load;
  int m = ee_load->GetNumberOfSegments();
//  int n = ee_load_->GetOptVarCount();

  SetDependentVariables({ee_load}, m);
  ee_load_ = ee_load;//std::dynamic_pointer_cast<EndeffectorLoad>(variables_.front());

  // build constant jacobian w.r.t lambdas
//  jac_ = Jacobian(m, n);

  Jacobian& jac = GetJacobianRefWithRespectTo(ee_load_->GetID());

  for (int k=0; k<m; ++k) {
    for (auto ee : ee_load->GetLoadValuesIdx(k).GetEEsOrdered()) {
      int idx = ee_load->IndexDiscrete(k,ee);
      jac.insert(k, idx) = 1.0;
    }
  }
}

//void
//ConvexityConstraint::UpdateVariables (const OptimizationVariables* opt_var)
//{
//  VectorXd lambdas = opt_var->GetVariables(ee_load_->GetID());
//  ee_load_->SetOptimizationParameters(lambdas);
//}

ConvexityConstraint::VectorXd
ConvexityConstraint::EvaluateConstraint () const
{
//  VectorXd g(jac_.rows());

  for (int k=0; k<num_constraints_; ++k) {

    double sum_k = 0.0;
    for (auto lambda : ee_load_->GetLoadValuesIdx(k).ToImpl())
      sum_k += lambda;

    g_(k) = sum_k; // sum equal to 1
  }

  return g_;
}

VecBound
ConvexityConstraint::GetBounds () const
{
  std::fill(bounds_.begin(), bounds_.end(), Bound(1.0, 1.0));
  return bounds_;

//  return VecBound(jac_.rows(), Bound(1.0, 1.0));
}

//ConvexityConstraint::Jacobian
//ConvexityConstraint::GetJacobianWithRespectTo (std::string var_set) const
//{
//  Jacobian jac; // empy matrix
//
//  if (var_set == ee_load_->GetID()) {
//    jac = jac_;
//  }
//
//  return jac;
//}

} /* namespace opt */
} /* namespace xpp */
