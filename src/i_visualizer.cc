/**
@file    i_visualizer.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Dec 7, 2016
@brief   Defines the IVisualizer class
 */

#include <xpp/opt/i_visualizer.h>
#include <xpp/opt/com_motion.h>
#include <xpp/opt/optimization_variables.h>

namespace xpp {
namespace opt {

IVisualizer::IVisualizer()
{
}

IVisualizer::~IVisualizer()
{
}


void
IVisualizer::SetMotionStructure (const MotionStructure& motion_structure)
{
  motion_structure_ = motion_structure;
}

void
IVisualizer::SetComMotion (const MotionPtrS& com_motion)
{
  com_motion_ = com_motion;
}

void
IVisualizer::SetOptimizationVariables (const OptimizationVariablesPtr& opt_variables)
{
  opt_variables_ = opt_variables;
}

void
IVisualizer::SetMotionParameters (const MotionParamsPtr& params)
{
  motion_params_ = params;
}

MotionStructure
IVisualizer::GetMotionStructure () const
{
  return motion_structure_;
}

const IVisualizer::MotionPtrS
IVisualizer::GetComMotion () const
{
  Eigen::VectorXd x_motion = opt_variables_->GetVariables(VariableNames::kSplineCoeff);
  com_motion_->SetCoefficients(x_motion);
  return com_motion_;
}

IVisualizer::VecFoothold
IVisualizer::GetContacts () const
{
  Eigen::VectorXd footholds = opt_variables_->GetVariables(VariableNames::kFootholds);
  return utils::ConvertEigToStd(footholds);
}

} // namespace opt
} // namespace xpp



