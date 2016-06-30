/**
 @file    nlp_structure.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Defines the class functions in NlpStructure
 */

#include <xpp/zmp/nlp_structure.h>

namespace xpp {
namespace zmp {

static constexpr int kDim2d = 2; // X,Y

NlpStructure::NlpStructure(int n_spline_coeff, int n_steps)
{
  Init(n_spline_coeff, n_steps);
}

NlpStructure::~NlpStructure ()
{
}

void
NlpStructure::Init(int n_spline_coeff, int n_steps)
{
  n_spline_coeff_ = n_spline_coeff;
  n_steps_ = n_steps;
  x_ = Eigen::VectorXd::Zero(n_spline_coeff_ + kDim2d*n_steps_);
}

int
NlpStructure::GetOptimizationVariableCount() const
{
  return x_.rows();
}

NlpStructure::VectorXd
NlpStructure::GetSplineCoefficients() const
{
  return x_.head(n_spline_coeff_);
}

NlpStructure::VectorXd
NlpStructure::GetOptimizationVariables () const
{
  return x_;
}

void
NlpStructure::SetAllVariables(const VectorXd& x_all)
{
  x_ = x_all;
}

void
NlpStructure::SetAllVariables(const Number* x_all)
{
  x_ = ConvertToEigen(x_all);
}

void
NlpStructure::SetSplineCoefficients(const VectorXd& x_abdc)
{
  x_.head(n_spline_coeff_) = x_abdc;
}

void
NlpStructure::SetFootholds(const VectorXd& footholds_xy)
{
  x_.middleRows(n_spline_coeff_, n_steps_*kDim2d) = footholds_xy;
}

NlpStructure::VectorXd
NlpStructure::GetFootholdsEig() const
{
  return x_.middleRows(n_spline_coeff_, kDim2d*n_steps_);
}

NlpStructure::VectorXd
NlpStructure::ConvertToEigen(const Number* x) const
{
  return Eigen::Map<const VectorXd>(x,GetOptimizationVariableCount());
}


} // namespace zmp
} // namespace xpp
