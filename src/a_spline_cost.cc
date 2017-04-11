/**
 @file    a_spline_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Definition of the ASplineCost, QuadraticSplineCost, SquaredSplineCost
 */

#include <xpp/opt/a_spline_cost.h>
#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {


ASplineCost::ASplineCost (const OptVarsPtr& opt_vars)
    :Cost(opt_vars)
{
}

QuadraticSplineCost::QuadraticSplineCost (const OptVarsPtr& opt_vars,
                                          const MatVec& mat_vec)
    :ASplineCost(opt_vars)
{
  matrix_vector_ = mat_vec;
  com_motion_    = std::dynamic_pointer_cast<BaseMotion>(opt_vars->GetSet("base_motion"));
}

QuadraticSplineCost::~QuadraticSplineCost ()
{
}

double
QuadraticSplineCost::EvaluateCost () const
{
  double cost = 0.0;
  VectorXd spline_coeff_ = com_motion_->GetXYSplineCoeffients();

  cost += spline_coeff_.transpose() * matrix_vector_.M * spline_coeff_;
  cost += matrix_vector_.v.transpose() * spline_coeff_;

  return cost;
}


QuadraticSplineCost::VectorXd
QuadraticSplineCost::EvaluateGradientWrt(std::string var_set)
{
  VectorXd grad;

  if (var_set == com_motion_->GetId())
    grad =  2.0 * matrix_vector_.M * com_motion_->GetXYSplineCoeffients();

  return grad;
}

//double
//SquaredSplineCost::EvaluateCost () const
//{
//  return (matrix_vector_.M*com_motion_->GetXYSplineCoeffients()
//          + matrix_vector_.v).norm();
//}

} /* namespace opt */
} /* namespace xpp */
