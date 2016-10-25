/**
 @file    cost_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Defines the class CostAdapter
 */

#include <xpp/opt/cost_adapter.h>
#include <iostream>

namespace xpp {
namespace opt {

CostAdapter::~CostAdapter ()
{
  // TODO Auto-generated destructor stub
}

CostAdapter::CostAdapter (const ConstraintPtr& constraint)
{
  constraint_ = constraint;
  int n_constraints = constraint_->GetNumberOfConstraints();

  std::cout << "n_constraints: " << n_constraints << std::endl;

  // treat all constraints equally by default
  weights_.resize(n_constraints);
  weights_.setOnes();
}

void
CostAdapter::UpdateVariables (const OptimizationVariables* x)
{
  constraint_->UpdateVariables(x);
}

double
CostAdapter::EvaluateCost () const
{
  VectorXd g = constraint_->EvaluateConstraint();
  return weights_.transpose()*g;
}

CostAdapter::VectorXd
CostAdapter::EvaluateGradientWrt (std::string var_set)
{
  VectorXd grad;

  std::cout << var_set << ": \n";
  AConstraint::Jacobian jac = constraint_->GetJacobianWithRespectTo(var_set);

  if (jac.rows() != 0)
    grad = jac.transpose()*weights_;

  std::cout << "jac.rows() = " << jac.rows() << std::endl;
  std::cout << "jac.cols() = " << jac.cols() << std::endl;
  std::cout << "weights_.rows() = " << weights_.rows() << std::endl;



  return grad;
}

} /* namespace opt */
} /* namespace xpp */
